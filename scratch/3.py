def f():
    print(1)

def g():
    print(2)

h = lambda: [f(), g()]

h()