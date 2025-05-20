import smartcar

sc = smartcar.SmartCar()
sc.drive_and_record_loop_voiceRec()
# It is important for cleaning up to set the SmartCar object to None
sc = None
