local dap, dapui = require("dap"), require("dapui")
dapui.setup()

require('dap-python').setup('/usr/bin/python')
table.insert(dap.configurations.python, {
  type = 'python',
  request = 'attach',
  name = 'Attach local',
  host = '127.0.0.1',
  port = 5678,
})

vim.keymap.set("n", "<leader>dc", ":lua require'dap'.continue()<CR>")
vim.keymap.set("n", "<leader>dn", ":lua require'dap'.step_over()<CR>") -- next
vim.keymap.set("n", "<leader>ds", ":lua require'dap'.step_into()<CR>") -- step
vim.keymap.set("n", "<leader>dr", ":lua require'dap'.step_out()<CR>") -- return

vim.keymap.set("n", "<leader>db", ":lua require'dap'.toggle_breakpoint()<CR>")
vim.keymap.set("n", "<leader>dB", ":lua require'dap'.set_breakpoint(vim.fn.input('Breakpoint condition: '))<CR>")
vim.keymap.set("n", "<leader>dl", ":lua require'dap'.repl.open()<CR>")

vim.keymap.set("n", "<leader>dp", ":lua require'dap'.run()<CR>")
vim.keymap.set("n", "<leader>du", ":lua require'dapui'.toggle()<CR>")

dap.listeners.after.event_initialized["dapui_config"] = function()
  dapui.open()
end
dap.listeners.before.event_terminated["dapui_config"] = function()
  dapui.close()
end
dap.listeners.before.event_exited["dapui_config"] = function()
  dapui.close()
end

require("nvim-dap-virtual-text").setup()
