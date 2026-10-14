# Expose PlatformIO launchers without selecting its Python for ROS.
_robu_path=":${PATH}:"
while [[ "$_robu_path" == *":$HOME/.platformio/penv/bin:"* ||
         "$_robu_path" == *":$HOME/.local/bin:"* ]]; do
    _robu_path="${_robu_path/":$HOME/.platformio/penv/bin:"/:}"
    _robu_path="${_robu_path/":$HOME/.local/bin:"/:}"
done
_robu_path="${_robu_path#:}"
_robu_path="${_robu_path%:}"
export PATH="$HOME/.local/bin:$_robu_path"
unset _robu_path
