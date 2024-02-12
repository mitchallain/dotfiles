#!/bin/bash

install_core_utils () {
    # gnome tweaks for theming and keyboard modifications
    sudo add-apt-repository universe

    if [ "$(lsb_release -rs)" = "22.04" ]; then
        sudo apt install \
            gnome-tweaks
    elif [ "$(lsb_release -rs)" = "20.04" ]; then
        sudo apt install \
            gnome-tweak-tool
    fi

    sudo apt install \
        bat \
        curl \
        htop \
        ripgrep \
        terminator \
        tmux \
        vim \
        xclip

    # bat is installed as batcat due to clash with another package
    mkdir -p ~/.local/bin
    if [ -f ~/usr/bin/batcat ] && [ ! -f ~/.local/bin/bat ]; then
        ln -s /usr/bin/batcat ~/.local/bin/bat
    fi
}

install_cpp_tools () {
    sudo apt install \
        cppcheck
}

install_clang () {
    sudo apt install \
        clang \
        clang-tools \
        clangd
}

install_dev_tools () {
    sudo apt install -y \
        direnv \
        entr
}




setup_python () {
    sudo apt install python3-pip
    python3 -m pip install --upgrade pip
}

install_ros1_tools () {
    sudo apt install \
        python-is-python3 \
        catkin-lint \
        python3-vcstool \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        build-essential \
        python3-catkin-tools
}

# apt show python3-colcon-common-extensions for a list of included extensions
install_ros2_tools () {
    sudo apt install \
        python3-colcon-common-extensions \
        python3-colcon-clean
}


# offline documentation browser
install_zeal () {
    if [ "$(lsb_release -rs)" = "22.04" ]; then
        sudo apt install zeal
    elif [ "$(lsb_release -rs)" = "20.04" ]; then
        sudo add-apt-repository ppa:zeal-developers/ppa
        sudo apt update
        sudo apt install zeal
    fi
}

# fzf - fuzzy command line finder
install_fzf () {
    git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
    ~/.fzf/install
}

install_forgit () {
    git clone git@github.com:wfxr/forgit.git ~/.forgit
}

# albert - alfred clone for linux (spotlight replacement)
# https://albertlauncher.github.io/installing/
# https://software.opensuse.org/download.html?project=home:manuelschneid3r&package=albert
install_alfred () {
    echo 'deb http://download.opensuse.org/repositories/home:/manuelschneid3r/xUbuntu_18.04/ /' \
        | sudo tee /etc/apt/sources.list.d/home:manuelschneid3r.list
    curl -fsSL https://download.opensuse.org/repositories/home:manuelschneid3r/xUbuntu_18.04/Release.key \
        | gpg --dearmor \
        | sudo tee /etc/apt/trusted.gpg.d/home_manuelschneid3r.gpg > /dev/null
    sudo apt update
    sudo apt install albert
}


# gitbatch - multiple git repository management tool
install_gitbatch () {
    gitbatchv="0.5.0"
    cd ~/Downloads || exit 1
    curl -OL https://github.com/isacikgoz/gitbatch/releases/download/vi${gitbatchv}/gitbatch_${gitbatchv}_linux_amd64.tar.gz
    tar xzf gitbatch_${gitbatchv}_linux_amd64.tar.gz
    sudo mv gitbatch /usr/local/bin
}

# quicktile - simple tiling
# http://ssokolow.com/quicktile/installation.html#b-install-sh-from-a-local-folder
# note: this will disable the built-in window moving shortcut
# quicktile is added as a daemon on next boot, can start manually after installation
# with `quicktile -d`
install_quicktile () {
    cd ~/Downloads || exit 1
    git clone git@github.com:ssokolow/quicktile.git
    cd quicktile || exit 1
    ./install.sh

    # TODO: disable built-in keybindings
    # gsettings set org.gnome.desktop.wm.keybindings move-to-side-w
}

# dotfiles setup
install_dotfiles () {
    git clone git@github.com:mitchallain/dotfiles.git ~/dotfiles
    cd ~/dotfiles || exit 1
    git submodule update --recursive --init
    ./install
}

# theme setup
install_whitesur_theme () {
    mkdir -p ~/dev && cd ~/dev || exit 1
    git clone git@github.com:vinceliuice/WhiteSur-gtk-theme.git
    git clone git@github.com:vinceliuice/WhiteSur-icon-theme.git
    git clone git@github.com:vinceliuice/WhiteSur-wallpapers.git
    cd WhiteSur-gtk-theme || exit 1
    ./install.sh  # note requires sudo
    sudo ./tweaks.sh -g -c Dark
    cd ../WhiteSur-icon-theme || exit 1
    ./install.sh
}

# nvim setup
# requires setup_python
install_nvim () {
    nvim_version="0.9.5"

    # remove symlink if already existing
    if [[ -L ~/bin/nvim ]]; then
        rm ~/bin/nvim
    fi

    # remove previous versions
    if [[ -f ~/bin/nvim.appimage ]]; then
        rm ~/bin/nvim.appimage
    fi

    wget -P ~/bin/ "https://github.com/neovim/neovim/releases/download/v$nvim_version/nvim.appimage"
    chmod u+x ~/bin/nvim.appimage
    ln -s ~/bin/nvim.appimage ~/bin/nvim

    # plug.vim
    curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
        https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim

   # this will retrieve a newer version than
   # "sudo apt install python3-pynvim"
    python3 -m pip install --user pynvim

    # install to update-alternatives
    sudo update-alternatives --install \
        /usr/bin/editor editor ~/bin/nvim 30
}

# https://askubuntu.com/a/1291854
install_node_16 () {
    sudo apt update
    curl -sL https://deb.nodesource.com/setup_16.x | sudo bash -
    sudo apt install -y nodejs
}


# nerd fonts meslo - da best
install_nf_meslo_font () {
    # rm ~/.local/share/fonts/Meslo*.ttf
    mkdir ~/.local/share/fonts
    wget -P ~/.local/share/fonts/ "https://github.com/ryanoasis/nerd-fonts/releases/download/v3.0.2/Meslo.zip"
    local cdir
    cdir=$(pwd)
    cd ~/.local/share/fonts/ || exit 1
    unzip Meslo.zip
    fc-cache -f -v
    rm Meslo.zip
    cd "$cdir" || exit 1
}

# sublime merge
# https://www.sublimemerge.com/docs/linux_repositories
install_sublime_merge () {
    wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg \
        | sudo apt-key add -
    sudo apt-get install apt-transport-https
    echo "deb https://download.sublimetext.com/ apt/stable/" \
        | sudo tee /etc/apt/sources.list.d/sublime-text.list
    sudo apt update
    sudo apt install sublime-merge
}

install_ros_noetic () {
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl # if you haven't already installed curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
        | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-desktop-full
}

# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
install_ros_humble () {
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu \
        $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt upgrade
    sudo apt install -y ros-humble-desktop ros-dev-tools
}


install_appimagelauncher () {
    sudo add-apt-repository ppa:appimagelauncher-team/stable
    sudo apt install appimagelauncher
}

install_obsidian () {
    obsidian_version="1.5.3"
    wget -P ~/Downloads/ "https://github.com/obsidianmd/obsidian-releases/releases/download/v$obsidian_version/Obsidian-$obsidian_version.AppImage"
    chmod +x ~/Downloads/Obsidian-$obsidian_version.AppImage
    ail-cli integrate ~/Downloads/Obsidian-$obsidian_version.Appimage
}

# newer versions of flameshot are not always available through apt
install_flameshot () {
    # https://flameshot.org/docs/installation/installation-linux/
    # https://github.com/flameshot-org/flameshot/releases
    wget -P ~/Downloads/ "https://github.com/flameshot-org/flameshot/releases/download/v12.1.0/flameshot-12.1.0-1.ubuntu-20.04.amd64.deb"
    sudo dpkg -i ~/Downloads/flameshot-12.1.0-1.ubuntu-20.04.amd64.deb
}
