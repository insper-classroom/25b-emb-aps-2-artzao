import pyvjoy

try:
    j = pyvjoy.VJoyDevice(1)  # Device 1
    print("vJoy Device 1 OK!")

    # Center axes ( vJoy range is 1..32768 usually)
    MID = 16384
    j.set_axis(pyvjoy.HID_USAGE_X, MID)
    j.set_axis(pyvjoy.HID_USAGE_Y, MID)
    print("Axes set to center")

except Exception as e:
    print("Erro ao inicializar vJoy:", e)
