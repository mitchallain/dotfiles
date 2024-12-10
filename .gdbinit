set auto-load safe-path /

# https://github.com/symphorien/nixseparatedebuginfod#manual-installation-without-the-module
set debuginfod enabled on

# Enables pretty-printing of objects, which formats the output of complex objects like structs and classes in a more human-readable way.
set print pretty on 
# Tells GDB to print objects with their runtime type (as opposed to static type), useful for polymorphic objects.
set print object on 
# Configures GDB to display static members when printing objects.
set print static-members on 
# When printing objects, GDB will include information about the virtual function table (vtable) if this is set on.
set print vtbl on 
# Enables demangling of C++ symbols to convert encoded function names into user-friendly names.
set print demangle on 
# Chooses the demangling style to use, with gnu-v3 being the style used by the GNU C++ compiler.
set demangle-style gnu-v3 
# Disables the assumption that strings are composed of 7-bit ASCII characters, allowing GDB to print the full 8-bit range which might be needed for UTF-8 or other 8-bit encodings.
set print sevenbit-strings off 
# Determines that when a fork occurs, GDB will follow the child process rather than the parent.
set follow-fork-mode child 
# Configures GDB to keep both parent and child processes under debugger control after a fork system call.
set detach-on-fork off 


# Register pretty printers for UE4 classes
python 
import sys

sys.path.append('/home/mallain/.config/Epic/GDBPrinters/')

try:
    from UE4Printers import register_ue4_printers
    register_ue4_printers(None)
    print('Registered pretty printers for UE4 classes')
except ImportError:
    pass
end

# Register pretty printers for Eigen classes
python
import sys
sys.path.insert(0, '/home/mallain/dotfiles/gdb/')
try:
    from eigen_printers import register_eigen_printers
    register_eigen_printers(None)
    print('Registered pretty printers for Eigen classes')
except ImportError:
    pass
end
