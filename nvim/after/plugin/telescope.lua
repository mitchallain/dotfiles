local builtin = require('telescope.builtin')
vim.keymap.set('n', '<leader>ff', builtin.find_files, {})
vim.keymap.set('n', '<leader>fg', builtin.live_grep, {})
vim.keymap.set('n', '<leader>fb', builtin.buffers, {})
vim.keymap.set('n', '<leader>fh', builtin.help_tags, {})
vim.keymap.set('n', '<leader>fm', builtin.marks, {})
vim.keymap.set('n', '<leader>fk', builtin.keymaps, {})
vim.keymap.set('n', '<leader>ft', builtin.current_buffer_tags, {})

-- vim.keymap.set('n', '<C-p>', builtin.find_files, {})
-- vim.api.nvim_set_keymap("n", "<C-p>", "<Cmd>lua require('telescope').extensions.frecency.frecency()<CR>", {noremap = true, silent = true})

-- BELOW is not yet working!
-- vim.api.nvim_set_keymap('n', '<leader>ff', 'builtin.find_files', {desc="Lists files in your current working directory, respects .gitignore"})
-- vim.api.nvim_set_keymap('n', '<C-p>', builtin.find_files, {desc="Lists files in your current working directory, respects .gitignore"})
-- vim.api.nvim_set_keymap('n', '<leader>fg', builtin.live_grep, {desc="Search for a string in your current working directory and get results live as you type, respects .gitignore."})
-- vim.api.nvim_set_keymap('n', '<leader>fb', builtin.buffers, {})
-- vim.api.nvim_set_keymap('n', '<leader>fh', builtin.help_tags, {})
-- vim.api.nvim_set_keymap('n', '<leader>fm', builtin.marks, {})
-- vim.api.nvim_set_keymap('n', '<leader>fk', builtin.keymaps, {})
-- vim.api.nvim_set_keymap('n', '<leader>ft', builtin.current_buffer_tags, {})


local telescope = require("telescope")
local telescope_config = require("telescope.config")

-- Clone the default Telescope configuration
local vimgrep_arguments = { unpack(telescope_config.values.vimgrep_arguments) }

-- I want to search in hidden/dot files.
table.insert(vimgrep_arguments, "--hidden")
-- I don't want to search in the `.git` directory.
table.insert(vimgrep_arguments, "--glob")
table.insert(vimgrep_arguments, "!**/.git/*")

telescope.setup({
	defaults = {
		-- `hidden = true` is not supported in text grep commands.
		vimgrep_arguments = vimgrep_arguments,
        path_display = { "smart" },
        selection_strategy = "reset",
	},
	pickers = {
		find_files = {
			-- `hidden = true` will still show the inside of `.git/` as it's not `.gitignore`d.
            -- couldn't quite get my rgignore file working ...
			-- find_command = { "rg", "--files", "--no-ignore-vcs", "--hidden", "--ignore-file", "~/.config/.rgignore" }
            find_command = { "rg", "--files", "--hidden", "--glob", "!**/.git/*" },
		},
	},
    extensions = {
        default_workspace = "CWD",
        frecency = {
            db_safe_mode = false,
        },
    }
})

-- WARN: these must be loaded after the setup() call above otherwise extensions will not be 
-- configured properly
-- https://github.com/nvim-telescope/telescope-fzf-native.nvim#telescope-setup-and-configuration
-- default options will override the telescope builtin sorter
telescope.load_extension('fzf')
telescope.load_extension('frecency')
