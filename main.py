# Play a sound when the robot is locked on to a target!
from networktables import NetworkTables
import random
import winsound
import time

possible_sounds = ["kaden.wav", "dylan.wav", "nathan.wav"]

def m_play_sound():
    print("playing sound...")
    winsound.PlaySound(random.choice(possible_sounds), winsound.SND_FILENAME)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    NetworkTables.initialize(server='roborio-2987-frc.local')
    sd = NetworkTables.getTable('SmartDashboard')

    while True:
        locked = sd.getBoolean('Manual Target Locked', False)
        turret_state = sd.getNumber("TurretState", 1.0)

        if locked == True and turret_state != 1.0:
            m_play_sound()
        else:
            time.sleep(.1)
