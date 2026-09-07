<template>
  <div class="nc-root" :class="{ 'nc-open': !minimized }">

    <button class="nc-toggle" @click="minimized = !minimized"
            :title="minimized ? 'Show nodeCam controls' : 'Hide'">
      {{ minimized ? "+" : "\u2013" }}
    </button>

    <div class="nc-panel" v-if="!minimized">
      <div class="nc-status">
        <span class="nc-dot" :class="dotClass"></span>
        <span class="nc-status-text">{{ statusText }}</span>
      </div>

      <button class="nc-btn" :class="{ on: st.picker }" @click="call('togglePicker')">
        {{ st.picker ? "Picker ON" : "Picker" }}
      </button>
      <button class="nc-btn" :disabled="!st.picker" @click="call('toggleNode')">
        Toggle node
      </button>
      <button class="nc-btn" @click="call('clearNodes')">
        Clear ({{ st.attached }})
      </button>
      <button class="nc-btn" @click="call('cycleSlot')">
        Cam {{ st.slot }}/{{ st.slotCount }}
      </button>
      <button class="nc-btn" :class="{ off: !st.active }" @click="call('toggleEnabled')">
        {{ st.active ? "Node lock ON" : "Steady cam" }}
      </button>

      <div class="nc-slider" v-for="s in sliders" :key="s.key">
        <span class="nc-slider-label">{{ s.label }}</span>
        <input class="nc-slider-input" type="range"
               :min="s.min" :max="s.max" :step="s.step"
               :value="st[s.key]" @input="setNum(s, $event.target.value)" />
        <span class="nc-slider-value">{{ fmt(s, st[s.key]) }}</span>
      </div>
    </div>

  </div>
</template>

<script setup>
import { ref, reactive, computed, onMounted, onUnmounted } from "vue"

const minimized = ref(false)
const st = reactive({
  active: true, running: false, steady: true, picker: false,
  attached: 0, slot: 1, slotCount: 3, fov: 65, moveSpeed: 1.1,
})

const sliders = [
  { key: "fov", label: "FOV", min: 10, max: 140, step: 1, dp: 0 },
  { key: "moveSpeed", label: "Spd", min: 0.5, max: 3, step: 0.05, dp: 2 },
]

let poll = null

function engine(script, cb) {
  if (!window.bngApi || !window.bngApi.engineLua) return
  if (cb) window.bngApi.engineLua(script, cb)
  else window.bngApi.engineLua(script)
}

function call(fn) {
  engine(`if nodeCamCore then nodeCamCore.${fn}() end`)
  setTimeout(refresh, 60)
}

function refresh() {
  engine("nodeCamCore and nodeCamCore.requestUIState()", d => {
    if (!d || typeof d !== "object") return
    st.active = d.active !== false
    st.running = !!d.running
    st.steady = !!d.steady
    st.attached = d.attached || 0
    st.slot = d.slot || 1
    st.slotCount = d.slotCount || 3
    if (d.settings) {
      st.picker = !!d.settings.picker
      for (const s of sliders) {
        if (!held[s.key] && d.settings[s.key] !== undefined) {
          st[s.key] = Number(d.settings[s.key])
        }
      }
    }
  })
}

// While a slider is being dragged, ignore the poll for that key so it cannot
// yank the handle back mid-adjustment.
const held = {}
const release = {}
function setNum(s, v) {
  st[s.key] = Number(v)
  held[s.key] = true
  if (release[s.key]) clearTimeout(release[s.key])
  release[s.key] = setTimeout(() => { held[s.key] = false }, 600)
  engine(`if nodeCamCore then nodeCamCore.set("${s.key}", ${st[s.key]}) end`)
}

function fmt(s, v) {
  const n = Number(v)
  return isFinite(n) ? n.toFixed(s.dp) : "-"
}

const statusText = computed(() => {
  if (!st.running) return "nodeCam inactive"
  if (!st.active) return "Steady cam"
  return st.steady ? "Steady" : `Locked: ${st.attached}`
})

const dotClass = computed(() => {
  if (!st.running || !st.active) return "off"
  return st.steady ? "steady" : "attached"
})

onMounted(() => {
  refresh()
  poll = setInterval(refresh, 500)
})

onUnmounted(() => {
  if (poll) clearInterval(poll)
})
</script>

<style scoped lang="scss">
/* The toggle is pinned to the top right and the panel grows down and to the
   left from it, so collapsing and expanding never moves the button. Collapsed,
   nothing but the button is painted: no panel, no background, no status dot. */
.nc-root {
  position: relative;
  width: 100%;
  height: 100%;
  color: #fff;
  font-size: 0.82rem;
  background: transparent;
  pointer-events: none;
}

.nc-toggle {
  position: absolute;
  top: 0;
  right: 0;
  z-index: 2;
  width: 1.4rem;
  height: 1.4rem;
  line-height: 1;
  border: 0;
  border-radius: 0.2rem;
  background: rgba(20, 22, 26, 0.55);
  color: #fff;
  cursor: pointer;
  pointer-events: auto;

  &:hover { background: rgba(60, 64, 72, 0.85); }
}

.nc-panel {
  position: absolute;
  top: 0;
  right: 0;
  display: flex;
  flex-direction: column;
  gap: 0.2rem;
  padding: 0.25rem;
  padding-right: 1.75rem;
  border-radius: 0.25rem;
  background: rgba(20, 22, 26, 0.62);
  pointer-events: auto;
}

.nc-status {
  display: flex;
  align-items: center;
  gap: 0.35rem;
  padding: 0.05rem 0.1rem 0.2rem;

  .nc-dot {
    width: 0.55rem;
    height: 0.55rem;
    border-radius: 50%;
    flex: 0 0 auto;
    background: #777;

    &.attached { background: #3ddc6a; }
    &.steady   { background: #f0a52a; }
    &.off      { background: #777; }
  }

  .nc-status-text {
    white-space: nowrap;
    opacity: 0.9;
  }
}

.nc-slider {
  display: flex;
  align-items: center;
  gap: 0.3rem;
  padding: 0.1rem 0.1rem 0;

  .nc-slider-label {
    flex: 0 0 1.7rem;
    opacity: 0.75;
  }

  .nc-slider-input {
    flex: 1 1 auto;
    min-width: 2.5rem;
    height: 0.9rem;
    accent-color: #3ddc6a;
    cursor: pointer;
  }

  .nc-slider-value {
    flex: 0 0 2.2rem;
    text-align: right;
    font-variant-numeric: tabular-nums;
    opacity: 0.85;
  }
}

.nc-btn {
  padding: 0.3rem 0.45rem;
  border: 0;
  border-radius: 0.2rem;
  background: rgba(255, 255, 255, 0.12);
  color: #fff;
  font-size: 0.82rem;
  cursor: pointer;
  text-align: left;
  white-space: nowrap;

  &:hover:not(:disabled) { background: rgba(255, 255, 255, 0.24); }
  &:disabled { opacity: 0.4; cursor: default; }
  &.on  { background: rgba(61, 220, 106, 0.32); }
  &.off { background: rgba(220, 61, 61, 0.32); }
}
</style>
