import { lua } from "@/bridge"

// Adds a button to the shared "Mods" tab of the pause menu. The card itself is
// NodeCamSettings.vue. Keep this file free of side effects at the top level:
// the runtime calls onUnload, re-evaluates the module, then calls onLoad again.

export async function onLoad() {
  await lua.extensions.ui_pause_actions.registerModButton({
    id: "nodecam-settings",
    tabId: "mods",
    label: "nodeCam",
    icon: "videocam",
    componentName: "/ui/ui-vue/mods/nodeCam/NodeCamSettings.vue",
  })
}

export async function onUnload() {
  await lua.extensions.ui_pause_actions.unregisterModButton("nodecam-settings")
}
