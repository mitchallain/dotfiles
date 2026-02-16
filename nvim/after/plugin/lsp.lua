require("trouble").setup()

-- Inserted from https://github.com/neovim/nvim-lspconfig#suggested-configuration
-- Mappings.
-- See `:help vim.diagnostic.*` for documentation on any of the below functions
local opts = { noremap = true, silent = true }
vim.keymap.set("n", "<leader>di", vim.diagnostic.open_float, opts)
-- nvim 0.11 replaced goto_prev/goto_next with vim.diagnostic.jump()
vim.keymap.set("n", "[d", function() vim.diagnostic.jump({ count = -1 }) end, opts)
vim.keymap.set("n", "]d", function() vim.diagnostic.jump({ count = 1 }) end, opts)

-- Testing trouble.nvim
-- vim.keymap.set("n", "<leader>q", vim.diagnostic.setloclist, opts)
vim.keymap.set("n", "<leader>qq", "<cmd>Trouble diagnostics toggle<cr>", opts)

-- https://github.com/hrsh7th/nvim-cmp/wiki/Example-mappings#luasnip
local has_words_before = function()
    local line, col = unpack(vim.api.nvim_win_get_cursor(0))
    return col ~= 0 and vim.api.nvim_buf_get_lines(0, line - 1, line, true)[1]:sub(col, col):match("%s") == nil
end

-- luasnip setup
local luasnip = require("luasnip")

-- nvim-cmp setup with lspkind
-- https://github.com/hrsh7th/nvim-cmp
local lspkind = require("lspkind")
lspkind.init()

local cmp = require("cmp")
cmp.setup({
    snippet = {
        expand = function(args)
            luasnip.lsp_expand(args.body)
        end,
    },
    mapping = {
        ["<C-n>"] = cmp.mapping(function(fallback)
            if cmp.visible() then
                cmp.select_next_item()
            elseif luasnip.expand_or_jumpable() then
                luasnip.expand_or_jump()
            elseif has_words_before() then
                cmp.complete()
            else
                fallback()
            end
        end, { "i", "s" }),

        ["<C-p>"] = cmp.mapping(function(fallback)
            if cmp.visible() then
                cmp.select_prev_item()
            elseif luasnip.jumpable(-1) then
                luasnip.jump(-1)
            else
                fallback()
            end
        end, { "i", "s" }),

        -- ["<C-n>"] = cmp.mapping.select_next_item({ behavior = cmp.SelectBehavior.Insert }),
        -- ["<C-p>"] = cmp.mapping.select_prev_item({ behavior = cmp.SelectBehavior.Insert }),

        ["<C-d>"] = cmp.mapping.scroll_docs(-4),
        ["<C-f>"] = cmp.mapping.scroll_docs(4),
        ["<C-e>"] = cmp.mapping.abort(),
        ["<C-y>"] = cmp.mapping(
            cmp.mapping.confirm({
                behavior = cmp.ConfirmBehavior.Insert,
                select = true,
            }),
            { "i", "c" }
        ),

        -- ["<tab>"] = cmp.config.disable,
        -- ["<tab>"] = cmp.mapping(function(fallback)
        --     fallback()
        -- end, { "i", "s" }),
        ["<tab>"] = vim.NIL,
        ["<S-tab>"] = vim.NIL,
    },
    sources = {
        -- note that order here dictates priority
        { name = "nvim_lua" },
        { name = "isort" },
        { name = "nvim_lsp" },
        { name = "path" },
        { name = "luasnip" },
        { name = "buffer", keyword_length = 5 },
        { name = "cmp_pandoc" },
    },
    formatting = {
        format = lspkind.cmp_format({
            with_text = true,
            menu = {
                buffer = "[buf]",
                nvim_lsp = "[lsp]",
                nvim_lua = "[api]",
                path = "[path]",
                luasnip = "[snip]",
                gh_issues = "[issues]",
                -- tn = "[tabnine]",
            },
        }),
    },

    experimental = {
        native_menu = false,
        ghost_text = true,
    },
})

-- Use buffer source for `/` (if you enabled `native_menu`, this won't work anymore).
-- cmp.setup.cmdline('/', {
--   mapping = cmp.mapping.preset.cmdline(),
--   sources = {
--     { name = 'buffer', keyword_length = 5 }
--   }
-- })

-- Use cmdline & path source for ':' (if you enabled `native_menu`, this won't work anymore).
-- cmp.setup.cmdline(':', {
--   mapping = cmp.mapping.preset.cmdline(),
--   sources = cmp.config.sources({
--     { name = 'path' }
--   }, {
--     { name = 'cmdline' }
--   })
-- })

-- Setup LSP capabilities for nvim-cmp integration
-- default_capabilities() extends vim.lsp.protocol.make_client_capabilities()
-- with completion capabilities that nvim-cmp can use
local capabilities = require("cmp_nvim_lsp").default_capabilities()

-- Set position encoding preference to prevent "multiple different position encodings" warning
-- The LSP spec uses general.positionEncodings (plural) to declare which encodings the client supports
-- List in order of preference: utf-16 first (for clangd/none-ls), then utf-8 as fallback
-- See: https://github.com/jose-elias-alvarez/null-ls.nvim/issues/428#issuecomment-997226723
capabilities.general = capabilities.general or {}
capabilities.general.positionEncodings = { "utf-16", "utf-8" }

-- Global configuration for all LSP servers
-- These capabilities will be merged into each server's config
-- ensuring nvim-cmp completion works across all language servers
vim.lsp.config('*', {
    capabilities = capabilities,
})

-- Configure servers with custom settings
-- nvim-lspconfig provides default cmd, filetypes, and root_markers
-- We only override what we've customized

vim.lsp.config.pyright = {
    settings = {
        pyright = {
            disableOrganizeImports = true,
        },
        python = {
            analysis = {
                ignore = { "*" },
            },
        },
    },
}
vim.lsp.enable('pyright')

-- ruff uses all defaults, no custom settings needed
vim.lsp.enable('ruff')

-- Customize clangd to exclude 'proto' filetype
vim.lsp.config.clangd = {
    filetypes = { "c", "cpp", "objc", "objcpp" }, -- remove proto
}
vim.lsp.enable('clangd')

vim.lsp.config.rust_analyzer = {
    settings = {
        ["rust-analyzer"] = {
            check = {
                -- override default usage of `cargo check` to use `cargo clippy`
                -- which encompasses check and adds lints
                command = "clippy",
                -- extra_args = { "--all-targets" },
            },
        },
    },
}
vim.lsp.enable('rust_analyzer')

-- ts_ls uses all defaults
vim.lsp.enable('ts_ls')

-- tombi uses all defaults (TOML language server)
vim.lsp.enable('tombi')

-- gopls uses all defaults
vim.lsp.enable('gopls')

vim.lsp.config.nil_ls = {
    settings = {
        ["nil"] = {
            formatting = {
                command = { "nixfmt" },
            },
        },
    },
}
vim.lsp.enable('nil_ls')

-- buf_ls uses all defaults
vim.lsp.enable('buf_ls')

vim.lsp.config.lua_ls = {
    settings = {
        Lua = {
            runtime = {
                -- Tell the language server which version of Lua you're using (most likely LuaJIT in the case of Neovim)
                version = "LuaJIT",
            },
            diagnostics = {
                -- Get the language server to recognize the `vim` global
                globals = { "vim" },
            },
            workspace = {
                -- Make the server aware of Neovim runtime files
                library = vim.api.nvim_get_runtime_file("", true),
                checkThirdParty = false,
            },
            -- Do not send telemetry data containing a randomized but unique identifier
            telemetry = {
                enable = false,
            },
        },
    },
}
vim.lsp.enable('lua_ls')

vim.lsp.config.yamlls = {
    settings = {
        redhat = {
            telemetry = {
                enable = false,
            },
        },
        yaml = {
            schemas = {
                ["https://raw.githubusercontent.com/CircleCI-Public/circleci-yaml-language-server/refs/heads/main/schema.json"] =
                "/.circleci/config.{yml,yaml}",
            },
        },
    },
}
vim.lsp.enable('yamlls')

-- bashls uses all defaults
vim.lsp.enable('bashls')

-- TODO: 'enable_cody = false' currently throws errors
-- require("sg").setup({
--     enable_cody = false,
--     on_attach = on_attach,
-- })

-- LspAttach autocommand replaces the old on_attach function
-- This runs whenever an LSP server attaches to a buffer
vim.api.nvim_create_autocmd('LspAttach', {
    group = vim.api.nvim_create_augroup('mallain-lsp-attach', { clear = true }),
    callback = function(args)
        local client = vim.lsp.get_client_by_id(args.data.client_id)
        local bufnr = args.buf

        -- Guard against nil client
        if not client then
            return
        end

        -- Server-specific customizations (grouped together)
        if client.name == "ruff" then
            -- Disable hover in favor of Pyright
            client.server_capabilities.hoverProvider = false
        end

        -- Mappings.
        -- See `:help vim.lsp.*` for documentation on any of the below functions
        -- Use client:supports_method() to check capabilities before setting keymaps
        local bufopts = { noremap = true, silent = true, buffer = bufnr }

        -- Conditional keymaps based on server method support
        if client:supports_method('textDocument/declaration') then
            vim.keymap.set("n", "gD", vim.lsp.buf.declaration, bufopts)
        end

        if client:supports_method('textDocument/definition') then
            vim.keymap.set("n", "gd", vim.lsp.buf.definition, bufopts)
        end

        if client:supports_method('textDocument/hover') then
            vim.keymap.set("n", "K", vim.lsp.buf.hover, bufopts)
        end

        if client:supports_method('textDocument/implementation') then
            vim.keymap.set("n", "gi", vim.lsp.buf.implementation, bufopts)
        end

        if client:supports_method('textDocument/signatureHelp') then
            vim.keymap.set("n", "<leader>k", vim.lsp.buf.signature_help, bufopts)
        end

        if client:supports_method('textDocument/typeDefinition') then
            vim.keymap.set("n", "<leader>D", vim.lsp.buf.type_definition, bufopts)
        end

        if client:supports_method('textDocument/rename') then
            vim.keymap.set("n", "<leader>rn", vim.lsp.buf.rename, bufopts)
        end

        if client:supports_method('textDocument/codeAction') then
            vim.keymap.set("n", "<leader>ca", vim.lsp.buf.code_action, bufopts)
        end

        if client:supports_method('textDocument/references') then
            -- Testing trouble.nvim
            vim.keymap.set("n", "<leader>sr", vim.lsp.buf.references, bufopts)
            vim.keymap.set("n", "gr", "<cmd>Trouble lsp_references toggle<cr>", bufopts)
        end

        if client:supports_method('textDocument/formatting') then
            -- Format whole file asynchronously
            vim.keymap.set("n", "<leader>fo", function()
                vim.lsp.buf.format({ async = true })
            end, bufopts)
        end

        if client:supports_method('textDocument/rangeFormatting') then
            -- Format a range from visual mode
            vim.keymap.set("v", "<leader>fo", vim.lsp.buf.format, bufopts)
        end

        -- Server-specific keymaps (not LSP methods, so check by name)
        if client.name == "clangd" then
            vim.keymap.set("n", "gsv", "<cmd>vsplit<cr><cmd>ClangdSwitchSourceHeader<cr>", bufopts)
            vim.keymap.set("n", "gss", "<cmd>split<cr><cmd>ClangdSwitchSourceHeader<cr>", bufopts)
            vim.keymap.set("n", "gs", "<cmd>ClangdSwitchSourceHeader<cr>", bufopts)
        end

        -- Custom navigation (goto split and goto vsplit)
        -- These are not LSP-specific, always set them
        vim.keymap.set("n", "gv", "<cmd>vert winc ]<cr>", bufopts)
        vim.keymap.set("n", "gl", "<cmd>winc ]<cr>", bufopts)
    end,
})

-- none-ls: Community fork of null-ls with nvim 0.11+ support
-- none-ls provides "null-ls" module for backward compatibility
local null_ls = require("null-ls")

null_ls.setup({
    sources = {
        -- use clangd over clang_format
        -- null_ls.builtins.formatting.clang_format,

        -- use ruff/pylint instead of:
        -- null_ls.builtins.diagnostics.pylint,
        -- null_ls.builtins.diagnostics.ruff,
        -- null_ls.builtins.diagnostics.flake8,
        -- null_ls.builtins.formatting.black.with({
        --     extra_args = { "--line-length", "99" },
        -- }),

        -- Shellcheck was removed from none-ls builtins
        -- Use bashls (bash-language-server) LSP instead for shell script diagnostics
        -- null_ls.builtins.diagnostics.shellcheck,
        -- null_ls.builtins.code_actions.shellcheck,

        -- TODO: retest
        -- null_ls.builtins.code_actions.gitsigns,
        -- null_ls.builtins.diagnostics.markdownlint,
        -- null_ls.builtins.formatting.yapf,

        null_ls.builtins.diagnostics.cmake_lint,
        null_ls.builtins.diagnostics.cppcheck.with({
            extra_args = { "--language=c++", "--inline-suppr" },
            temp_dir = "/tmp",
        }),
        null_ls.builtins.formatting.cmake_format,

        -- had to npm install --global prettier
        null_ls.builtins.formatting.prettier.with({
            extra_args = { "--print-width", "80", "--prose-wrap", "always" },
            filetypes = { "html", "json", "yaml", "markdown", "xml" }, -- overwrite to add xml via plugin
        }),
        null_ls.builtins.formatting.stylua.with({
            extra_args = { "--indent-type", "Spaces" },
        }),
    },

    -- debug = true,
    log_level = "warn",
    -- log_level = "info",
    -- log_level = "debug",
})

local format_diagnostic = function(diagnostic)
    return string.format("%s (%s)", diagnostic.message, diagnostic.code)
end

-- Shared diagnostic display configuration
-- Used by both vim.diagnostic.config() and toggle_lsp_diagnostics plugin
local diagnostic_opts = {
    severity_sort = true,
    underline = true,
    update_in_insert = false,
    virtual_text = {                -- Inline text at end of line with diagnostic
        spacing = 4,
        prefix = "●",
    },
    float = {                       -- Floating window (shown with <leader>di / vim.diagnostic.open_float)
        source = true,              -- Always show which LSP server reported the diagnostic
        format = format_diagnostic, -- Include diagnostic code in the message
    },
}

-- Configure how LSP diagnostics are displayed globally
vim.diagnostic.config(diagnostic_opts)

-- Plugin that allows toggling diagnostics on/off with keymaps
-- Uses the same diagnostic_opts to ensure toggling restores the correct display settings
-- start_on currently does not work
require("toggle_lsp_diagnostics").init(vim.tbl_extend('force', diagnostic_opts, {
    start_on = true,
}))

vim.keymap.set("n", "<leader>dd", "<Plug>(toggle-lsp-diag)")
vim.keymap.set("n", "<leader>dT", "<Plug>(toggle-lsp-diag-vtext)")
vim.keymap.set("n", "<leader>de", vim.diagnostic.enable)

-- require('lspfuzzy').setup {}
