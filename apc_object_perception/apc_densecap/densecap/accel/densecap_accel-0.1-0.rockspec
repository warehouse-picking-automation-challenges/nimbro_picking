
package = "densecap_accel"
version = "0.1-0"

source = {
    url = "..."
}

description = {
    summary = "Fast subroutines for DenseCap",
    detailed = [[
    ]],
    license = "BSD"
}

dependencies = {
    "torch >= 7.0"
}

build = {
    type = "command",
    build_command = [[
cmake -E make_directory buildrock && cd buildrock && cmake .. -DLUALIB=$(LUALIB) -DLUA_INCDIR="$(LUA_INCDIR)" -DLUA_LIBDIR="$(LUA_LIBDIR)"  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$(LUA_BINDIR)/.." -DCMAKE_INSTALL_PREFIX="$(PREFIX)" && $(MAKE)
    ]],
    install_command = "cd buildrock && $(MAKE) install"
}
