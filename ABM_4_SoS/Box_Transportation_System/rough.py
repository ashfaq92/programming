import utils
utils.DEBUG_MODE = True
utils.MAX_STEPS = 1000  # Increase to see meaningful results

from diagnostic import run_diagnostic
run_diagnostic()