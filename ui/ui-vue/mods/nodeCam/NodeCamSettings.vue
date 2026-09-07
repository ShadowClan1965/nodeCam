<template>
  <div class="nc-options">

    <div class="options-header">
      <div class="header-title">nodeCam</div>
      <div class="header-toggle">
        <BngSwitch v-model="isActive" @update:modelValue="setEnabled">
          Follow nodes
        </BngSwitch>
      </div>
    </div>

    <div class="nc-status" :class="statusClass">
      <span class="nc-dot"></span>
      <span>{{ statusText }}</span>
      <span class="nc-status-sub" v-if="state.totalNodes">
        cam {{ state.slot }} of {{ state.slotCount }} &middot;
        {{ state.totalNodes }} nodes on this vehicle
      </span>
    </div>

    <div class="options-list-scroll">

      <details
        v-for="(category, catIndex) in schema"
        :key="category.name"
        class="category-block"
        :open="catIndex === 0"
      >
        <summary class="category-title">{{ category.name }}</summary>

        <div class="category-items">
          <div
            v-for="item in category.items"
            :key="item.id"
            class="options-item-row"
            @mouseenter="hint = item.hint"
            @mouseleave="hint = ''"
          >
            <div class="item-label">{{ item.label }}</div>

            <div class="item-control" v-if="item.type === 'bool'">
              <BngSwitch
                :modelValue="!!settings[item.id]"
                @update:modelValue="v => setSetting(item.id, v)"
              />
            </div>

            <div class="item-control item-slider" v-else>
              <BngSlider
                :modelValue="Number(settings[item.id])"
                :min="item.min"
                :max="item.max"
                :step="item.step"
                @update:modelValue="v => setSetting(item.id, v)"
              />
              <div class="item-value">{{ fmt(settings[item.id], item.step) }}</div>
            </div>
          </div>
        </div>
      </details>

    </div>

    <div class="options-footer">
      <BngButton :accent="ACCENTS.outlined" @click="resetAll">Reset to defaults</BngButton>
      <div class="nc-hint">{{ hint || "Hover over a setting to see details." }}</div>
    </div>

  </div>
</template>

<script setup>
import { ref, reactive, computed, onMounted, onUnmounted } from "vue"
import { BngButton, BngSwitch, BngSlider, ACCENTS } from "@/common/components/base"

const hint = ref("")
const isActive = ref(true)
const settings = reactive({})
const state = reactive({ slot: 1, slotCount: 3, totalNodes: 0, attached: 0, steady: true, running: false })

let poll = null

// Mirrors nodeCamCore.defaults. Anything here is read and written through
// nodeCamCore.set(), so clamping and persistence stay on the Lua side.
const schema = [
  {
    name: "Camera",
    items: [
      { id: "fov", label: "Field of view", type: "num", min: 10, max: 140, step: 1,
        hint: "nodeCam sets the FOV itself every frame, so the game's zoom keys do not apply here." },
      { id: "moveSpeed", label: "Camera move speed", type: "num", min: 0.5, max: 3, step: 0.05,
        hint: "How fast the movement keys push the anchor around, in metres per second." },
      { id: "fastMultiplier", label: "Fast modifier", type: "num", min: 1, max: 20, step: 0.5,
        hint: "Speed multiplier while the fast movement modifier is held." },
      { id: "slotCount", label: "Saved cameras per vehicle", type: "num", min: 1, max: 6, step: 1,
        hint: "How many camera slots the cycle keybind steps through." },
    ],
  },
  {
    name: "Look",
    items: [
      { id: "lookSensitivity", label: "Mouse sensitivity", type: "num", min: 0.01, max: 3, step: 0.01,
        hint: "Scales raw mouse deltas. The raw values are far too hot to use directly." },
      { id: "keyLookSpeed", label: "Pad / key look speed", type: "num", min: 0.05, max: 10, step: 0.05,
        hint: "Look speed for analog stick and keyboard look, in radians per second." },
      { id: "invertYaw", label: "Invert mouse yaw", type: "bool",
        hint: "Affects the mouse only." },
      { id: "invertPitch", label: "Invert mouse pitch", type: "bool",
        hint: "Affects the mouse only." },
      { id: "invertPadYaw", label: "Invert pad yaw", type: "bool",
        hint: "Pad and keyboard look arrive on different fields from the mouse and need their own signs." },
      { id: "invertPadPitch", label: "Invert pad pitch", type: "bool",
        hint: "Pad and keyboard look arrive on different fields from the mouse and need their own signs." },
    ],
  },
  {
    name: "Node picker",
    items: [
      { id: "picker", label: "Show node picker", type: "bool",
        hint: "Draws nearby nodes so you can aim at them. Reads every node each frame, so leave it off while driving." },
      { id: "pickRadius", label: "Pick radius", type: "num", min: 0.3, max: 8, step: 0.1,
        hint: "How far from the camera nodes are drawn, in metres." },
      { id: "pickSpread", label: "Crosshair tolerance", type: "num", min: 0.005, max: 0.3, step: 0.005,
        hint: "Larger is more forgiving when aiming at a node." },
      { id: "maxDrawnNodes", label: "Max drawn nodes", type: "num", min: 10, max: 2000, step: 10,
        hint: "Cap on nodes drawn at once. The nearest are kept when the cap bites." },
    ],
  },
  {
    name: "Movement bounds",
    items: [
      { id: "boundsEnabled", label: "Leash camera to vehicle", type: "bool",
        hint: "Off lets the anchor go anywhere. On keeps it within the margin below." },
      { id: "boundsMargin", label: "Bounds margin", type: "num", min: 0, max: 50, step: 0.5,
        hint: "How far past the vehicle body the anchor may travel, in metres." },
    ],
  },
  {
    name: "Softness",
    items: [
      { id: "softness", label: "Softness", type: "num", min: 0, max: 1, step: 0.01,
        hint: "0 is a rigid weld. Above 0 blends in virtual springs for a looser, floatier mount." },
      { id: "stiffness", label: "Spring stiffness", type: "num", min: 1, max: 5000, step: 10,
        hint: "Only used when softness is above 0." },
      { id: "damping", label: "Spring damping", type: "num", min: 0, max: 200, step: 1,
        hint: "Higher settles faster. Too low will wobble." },
      { id: "maxSag", label: "Max sag", type: "num", min: 0, max: 2, step: 0.01,
        hint: "Hard limit on how far the soft camera may drift from the rigid answer, in metres." },
    ],
  },
  {
    name: "Crash handling",
    items: [
      { id: "outlierResidual", label: "Outlier floor", type: "num", min: 0.01, max: 1, step: 0.01,
        hint: "A node this far from where the rigid fit expects it is dropped. This is what stops the view flying off when a node tears away." },
      { id: "outlierMedianScale", label: "Outlier median scale", type: "num", min: 1, max: 10, step: 0.1,
        hint: "Also drop anything this many times worse than the median node." },
      { id: "maxDropFraction", label: "Max dropped at once", type: "num", min: 0, max: 0.9, step: 0.05,
        hint: "Never discard more than this share of the set in one frame." },
    ],
  },
  {
    name: "Misc",
    items: [
      { id: "quiet", label: "Quiet logging", type: "bool",
        hint: "Suppresses nodeCam's info lines in the console. Warnings and errors always get through." },
    ],
  },
]

function fmt(v, step) {
  const n = Number(v)
  if (!isFinite(n)) return "-"
  if (step >= 1) return String(Math.round(n))
  return n.toFixed(step >= 0.1 ? 1 : 2)
}

const statusText = computed(() => {
  if (!state.running) return "Not active - press C until you reach nodeCam"
  if (!isActive.value) return "Steady cam - node following off"
  if (state.steady) return "Steady view - no nodes picked"
  return `Attached to ${state.attached} nodes`
})

const statusClass = computed(() => {
  if (!state.running || !isActive.value) return "nc-off"
  return state.steady ? "nc-steady" : "nc-attached"
})

function callLua(script, cb) {
  if (!window.bngApi || !window.bngApi.engineLua) return
  if (cb) window.bngApi.engineLua(script, cb)
  else window.bngApi.engineLua(script)
}

function refresh() {
  callLua("nodeCamCore and nodeCamCore.requestUIState()", data => {
    if (!data || typeof data !== "object") return
    if (data.settings) Object.assign(settings, data.settings)
    isActive.value = data.active !== false
    state.slot = data.slot || 1
    state.slotCount = data.slotCount || 3
    state.totalNodes = data.totalNodes || 0
    state.attached = data.attached || 0
    state.steady = !!data.steady
    state.running = !!data.running
  })
}

function setSetting(key, value) {
  settings[key] = value
  const lv = typeof value === "boolean" ? (value ? "true" : "false") : Number(value)
  callLua(`if nodeCamCore then nodeCamCore.set("${key}", ${lv}) end`)
}

function setEnabled(v) {
  isActive.value = v
  callLua(`if nodeCamCore then nodeCamCore.set("enabled", ${v ? "true" : "false"}) end`)
}

function resetAll() {
  callLua("if nodeCamCore then nodeCamCore.resetSettings() end")
  setTimeout(refresh, 100)
}

onMounted(() => {
  refresh()
  poll = setInterval(refresh, 1000)
})

onUnmounted(() => {
  if (poll) clearInterval(poll)
})
</script>

<style scoped lang="scss">
.nc-options {
  display: flex;
  flex-direction: column;
  height: 100%;
  min-height: 0;
}

.options-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 0.5rem 0.75rem;
  border-bottom: 1px solid rgba(255, 255, 255, 0.12);

  .header-title {
    font-size: 1.25rem;
    font-weight: 600;
  }
}

.nc-status {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  padding: 0.4rem 0.75rem;
  font-size: 0.85rem;
  opacity: 0.9;

  .nc-dot {
    width: 0.6rem;
    height: 0.6rem;
    border-radius: 50%;
    background: #888;
    flex: 0 0 auto;
  }

  .nc-status-sub {
    margin-left: auto;
    opacity: 0.6;
    font-size: 0.78rem;
  }

  &.nc-attached .nc-dot { background: #3ddc6a; }
  &.nc-steady .nc-dot { background: #f0a52a; }
  &.nc-off .nc-dot { background: #777; }
}

.options-list-scroll {
  flex: 1 1 auto;
  min-height: 0;
  overflow-y: auto;
  padding: 0 0.5rem;
}

.category-block {
  margin: 0.35rem 0;
  border-radius: 0.3rem;
  background: rgba(255, 255, 255, 0.04);
}

.category-title {
  cursor: pointer;
  padding: 0.45rem 0.6rem;
  font-weight: 600;
  user-select: none;
}

.category-items {
  padding: 0.15rem 0.6rem 0.5rem;
}

.options-item-row {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.3rem 0;

  .item-label {
    flex: 1 1 auto;
    font-size: 0.9rem;
  }

  .item-control {
    flex: 0 0 auto;
  }

  .item-slider {
    display: flex;
    align-items: center;
    gap: 0.5rem;
    width: 16rem;

    > :first-child { flex: 1 1 auto; }
  }

  .item-value {
    flex: 0 0 3.2rem;
    text-align: right;
    font-variant-numeric: tabular-nums;
    opacity: 0.8;
  }
}

.options-footer {
  display: flex;
  align-items: center;
  gap: 0.75rem;
  padding: 0.5rem 0.75rem;
  border-top: 1px solid rgba(255, 255, 255, 0.12);

  .nc-hint {
    flex: 1 1 auto;
    font-size: 0.8rem;
    font-style: italic;
    opacity: 0.65;
  }
}
</style>
