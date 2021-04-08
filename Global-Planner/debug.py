import time

_on = True

def on():
    global _on
    _on = True

def off():
    global _on
    _on = False

def log(message):
    if _on: print(f"[DEBUG]: {message}")

def profiled(fn):
    def wrapper(*args, **kwargs):
        start = time.perf_counter()
        out = fn(*args, **kwargs)
        if _on: print(f"{fn} took {time.perf_counter() - start} sec")
        return out

    return wrapper

def conditional(fn):
    def wrapper(*args, **kwargs):
        if _on: return fn(*args, **kwargs)

    return wrapper
