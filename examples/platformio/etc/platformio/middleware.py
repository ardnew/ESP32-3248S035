Import("env")

def cflags(env, node):
    return env.Object(
        node,
        CFLAGS=env['CFLAGS'] + 
            ['-std=gnu99']
    )

def cxxflags(env, node):
    return env.Object(
        node,
        CXXFLAGS=env['CXXFLAGS'] + 
            ['-std=gnu++17']
    )

env.AddBuildMiddleware(cflags, "*.c")
env.AddBuildMiddleware(cxxflags, "*.cpp")
