-- Loaded automatically when a level starts. Brings up the nodeCam state module
-- so keybinds and console commands have something to talk to.
--
-- The camera mode loads this on demand as well, so nodeCam still works even if
-- this hook never fires.

pcall(function()
  extensions.load('nodeCamCore')
end)
