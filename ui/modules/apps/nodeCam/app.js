// Angular build of the nodeCam control app, for the pre-redesign UI. app.vue is
// the same thing for the new one; the game uses whichever its renderer supports.

angular.module('beamng.apps')
.directive('nodeCamControls', ['$interval', function ($interval) {
  return {
    template:
      '<div style="position:relative;width:100%;height:100%;color:#fff;font-size:0.82em;background:transparent;pointer-events:none;">' +
        '<button class="nc-toggle" ng-click="minimized = !minimized">{{ minimized ? "+" : "\u2013" }}</button>' +
        '<div class="nc-panel" ng-if="!minimized">' +
          '<div class="nc-status">' +
            '<span class="nc-dot" ng-style="{background: dotColor()}"></span>' +
            '<span style="white-space:nowrap;opacity:0.9;">{{ statusText() }}</span>' +
          '</div>' +
          '<button class="ncb" ng-click="call(\'togglePicker\')" ng-style="{background: st.picker ? \'rgba(61,220,106,0.32)\' : \'rgba(255,255,255,0.12)\'}">{{ st.picker ? "Picker ON" : "Picker" }}</button>' +
          '<button class="ncb" ng-click="call(\'toggleNode\')" ng-disabled="!st.picker">Toggle node</button>' +
          '<button class="ncb" ng-click="call(\'clearNodes\')">Clear ({{ st.attached }})</button>' +
          '<button class="ncb" ng-click="call(\'cycleSlot\')">Cam {{ st.slot }}/{{ st.slotCount }}</button>' +
          '<button class="ncb" ng-click="call(\'toggleEnabled\')" ng-style="{background: st.active ? \'rgba(255,255,255,0.12)\' : \'rgba(220,61,61,0.32)\'}">{{ st.active ? "Node lock ON" : "Steady cam" }}</button>' +
          '<div class="nc-slider" ng-repeat="s in sliders">' +
            '<span style="flex:0 0 26px;opacity:0.75;">{{ s.label }}</span>' +
            '<input class="nc-slider-input" type="range" min="{{ s.min }}" max="{{ s.max }}" step="{{ s.step }}" ' +
              'ng-model="st[s.key]" ng-change="setNum(s, st[s.key])" />' +
            '<span style="flex:0 0 34px;text-align:right;opacity:0.85;">{{ fmt(s, st[s.key]) }}</span>' +
          '</div>' +
        '</div>' +
      '</div>',
    replace: true,
    link: function (scope, element) {
      'use strict'

      scope.minimized = false
      scope.st = {
        active: true, running: false, steady: true, picker: false,
        attached: 0, slot: 1, slotCount: 3, fov: 65, moveSpeed: 1.1,
      }

      scope.sliders = [
        { key: 'fov', label: 'FOV', min: 10, max: 140, step: 1, dp: 0 },
        { key: 'moveSpeed', label: 'Spd', min: 0.5, max: 3, step: 0.05, dp: 2 },
      ]

      // While a slider is being dragged, ignore the poll for that key so it
      // cannot yank the handle back mid-adjustment.
      var held = {}
      var release = {}

      scope.setNum = function (s, v) {
        var n = Number(v)
        scope.st[s.key] = n
        held[s.key] = true
        if (release[s.key]) clearTimeout(release[s.key])
        release[s.key] = setTimeout(function () { held[s.key] = false }, 600)
        bngApi.engineLua('if nodeCamCore then nodeCamCore.set("' + s.key + '", ' + n + ') end')
      }

      scope.fmt = function (s, v) {
        var n = Number(v)
        return isFinite(n) ? n.toFixed(s.dp) : '-'
      }

      // Toggle pinned top right, panel grows down and left from it, so the
      // button never moves between states and nothing is painted when closed.
      var style = document.createElement('style')
      style.textContent =
        '.nc-toggle{position:absolute;top:0;right:0;z-index:2;width:22px;height:22px;line-height:1;' +
          'border:0;border-radius:3px;background:rgba(20,22,26,0.55);color:#fff;cursor:pointer;pointer-events:auto;}' +
        '.nc-toggle:hover{background:rgba(60,64,72,0.85);}' +
        '.nc-panel{position:absolute;top:0;right:0;display:flex;flex-direction:column;gap:3px;' +
          'padding:4px 28px 4px 4px;border-radius:4px;background:rgba(20,22,26,0.62);pointer-events:auto;}' +
        '.nc-status{display:flex;align-items:center;gap:5px;padding:1px 2px 3px;}' +
        '.nc-dot{width:9px;height:9px;border-radius:50%;flex:0 0 auto;}' +
        '.nc-slider{display:flex;align-items:center;gap:4px;padding:2px 1px 0;font-size:0.82em;}' +
        '.nc-slider-input{flex:1 1 auto;min-width:40px;height:14px;accent-color:#3ddc6a;cursor:pointer;}' +
        '.ncb{padding:4px 6px;border:0;border-radius:3px;background:rgba(255,255,255,0.12);' +
          'color:#fff;font-size:0.82em;cursor:pointer;text-align:left;white-space:nowrap;}' +
        '.ncb:hover:not([disabled]){background:rgba(255,255,255,0.24);}' +
        '.ncb[disabled]{opacity:0.4;cursor:default;}'
      element[0].appendChild(style)

      scope.statusText = function () {
        if (!scope.st.running) return 'nodeCam inactive'
        if (!scope.st.active) return 'Steady cam'
        return scope.st.steady ? 'Steady' : 'Locked: ' + scope.st.attached
      }

      scope.dotColor = function () {
        if (!scope.st.running || !scope.st.active) return '#777'
        return scope.st.steady ? '#f0a52a' : '#3ddc6a'
      }

      function refresh() {
        bngApi.engineLua('nodeCamCore and nodeCamCore.requestUIState()', function (d) {
          if (!d || typeof d !== 'object') return
          scope.$evalAsync(function () {
            scope.st.active = d.active !== false
            scope.st.running = !!d.running
            scope.st.steady = !!d.steady
            scope.st.attached = d.attached || 0
            scope.st.slot = d.slot || 1
            scope.st.slotCount = d.slotCount || 3
            if (d.settings) {
              scope.st.picker = !!d.settings.picker
              scope.sliders.forEach(function (s) {
                if (!held[s.key] && d.settings[s.key] !== undefined) {
                  scope.st[s.key] = Number(d.settings[s.key])
                }
              })
            }
          })
        })
      }

      scope.call = function (fn) {
        bngApi.engineLua('if nodeCamCore then nodeCamCore.' + fn + '() end')
        setTimeout(refresh, 60)
      }

      refresh()
      var poll = $interval(refresh, 500)

      scope.$on('$destroy', function () {
        $interval.cancel(poll)
      })
    }
  }
}]);
