set auto-load safe-path /

# https://github.com/symphorien/nixseparatedebuginfod#manual-installation-without-the-module
set debuginfod enabled on

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
