import smartcar 
import paho.mqtt.client as mqtt
import cv2
import numpy as np
from PIL import Image, ImageDraw
from pycoral.adapters import common, detect
from pycoral.utils.edgetpu import make_interpreter
import socket
import base64

# Configuración MQTT
HOST = 'localhost'    
PORT = 50007              

model = "/home/mendel/SmartCar/red3_edgetpu.tflite"
CLASSES = ["red", "redyellow", "green", "yellow", "off", "Person"]
COLORS = ['red', 'orange', 'green', 'yellow', 'black', 'grey']
THRESHOLD = 0.25
interpreter = make_interpreter(model)
interpreter.allocate_tensors()

tl = "TL15"
mode = "standby"
phasemqtt = "red"
phasecnn = "red"
current_action = "Idle"
sound_detected = "None"

def sendframe(frame):
    framestream = cv2.resize(frame, (320, 240))
    ret, buffer = cv2.imencode('.jpg', framestream)
    b64 = 'data:image/jpeg;base64,' + base64.b64encode(buffer).decode()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.sendall(bytes(b64, 'utf-8'))

def on_message(client, userdata, message):
    global mode, phasemqtt, current_action, sound_detected
    topic_str = message.topic
    payload_str = message.payload.decode('ASCII')
    print(f"Received: topic={topic_str}  payload={payload_str}")

    if topic_str == "SMARTCAR_control/mode":
        if payload_str in ["standby", "lanedetection", "tlviamqtt", "tlviacnn", "quit"]:
            mode = payload_str
            current_action = f"Mode set to {mode}"
    elif topic_str == tl:
        phasemqtt = payload_str
    elif topic_str == "SMARTCAR_control/manual_action":
        # Comandos manuales desde dashboard: go, stop, backwards
        action = payload_str.lower()
        if action == "go":
            mode = "lanedetection"
            current_action = "Manual: Go"
        elif action == "stop":
            mode = "standby"
            current_action = "Manual: Stop"
        elif action == "backwards":
            mode = "standby"
            current_action = "Manual: Backwards (stop for safety)"
        # Podrías implementar movimiento hacia atrás si tu SmartCar lo soporta
    elif topic_str == "SMARTCAR_control/sound":
        sound_detected = payload_str
        current_action = f"Sound detected: {sound_detected}"

def publish_mqtt(client, mode, speed, steer, campan, camtilt, tlphase, vcc):
    client.publish("SMARTCAR_status/mode", mode, retain=False)
    client.publish("SMARTCAR_status/speed", speed, retain=False)
    client.publish("SMARTCAR_status/steer", steer, retain=False)
    client.publish("SMARTCAR_status/campan", campan, retain=False)
    client.publish("SMARTCAR_status/camtilt", camtilt, retain=False)
    client.publish("SMARTCAR_status/tlphase", tlphase, retain=False)
    client.publish("SMARTCAR_status/vcc", vcc, retain=False)
    client.publish("SMARTCAR_status/action", current_action, retain=False)  # Acción para el dashboard
    client.publish("SMARTCAR_status/sound", sound_detected, retain=False)   # Sonido detectado para el dashboard
    client.publish("SMARTCAR_status/alive", "true", retain=False)

def analyze_draw_objects(draw, objs):
    objects = []
    for obj in objs:
        if obj.id > len(CLASSES)-1:
            print("Unknown class id:", obj.id)
        else:
            bbox = obj.bbox
            draw.rectangle([(bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax)],
                           outline=COLORS[obj.id], width=2)
            draw.text((bbox.xmin, bbox.ymin),
                      '%s (%.0f%%)' % (CLASSES[obj.id], obj.score*100),
                      fill=COLORS[obj.id])
            objects.append(CLASSES[obj.id])
    return objects

def main():
    global phasecnn, current_action
    client = mqtt.Client()
    client.connect("localhost", 1883, 60)
    client.on_message = on_message
    client.subscribe("SMARTCAR_control/mode")
    client.subscribe(tl)
    client.subscribe("SMARTCAR_control/manual_action")  # Suscripción para comandos manuales
    client.subscribe("SMARTCAR_control/sound")          # Suscripción para sonido simulado
    client.loop_start()

    sc = smartcar.SmartCar(use_ultrasonic=True, use_local_window=False)
    try:
        while not sc.quit:
            phase2send = "off"
            sc.handle_sensors()

            if mode == "lanedetection":
                if sc.speed < 40:
                    sc.speed += 1
                sc.lane_detection()
                sc.handle_actuators()
                sendframe(sc.frame)
                current_action = "Lane Detection Mode"

            elif mode == "tlviamqtt":
                if phasemqtt in ["red", "redyellow"]:
                    sc.speed = 0
                elif phasemqtt == "yellow":
                    sc.speed = 30
                elif phasemqtt == "green":
                    sc.speed = 40
                sc.lane_detection()
                sc.handle_actuators()
                sendframe(sc.frame)
                phase2send = phasemqtt
                current_action = f"TL via MQTT: {phasemqtt}"

            elif mode == "tlviacnn":
                image = Image.fromarray(cv2.cvtColor(sc.frame, cv2.COLOR_BGR2RGB))
                draw = ImageDraw.Draw(image)
                _, scale = common.set_resized_input(interpreter, image.size, lambda size: image.resize(size, Image.ANTIALIAS))
                interpreter.invoke()
                objs = detect.get_objects(interpreter, THRESHOLD, scale)
                if objs:
                    objects = analyze_draw_objects(draw, objs)
                else:
                    objects = []
                frameout = np.array(image)
                sc.frame = cv2.cvtColor(frameout, cv2.COLOR_RGB2BGR)

                phasecnn = "off"
                sc.speed = 40
                if "red" in objects:
                    phasecnn = "red"
                    sc.speed = 0
                elif "redyellow" in objects:
                    phasecnn = "redyellow"
                    sc.speed = 0
                elif "yellow" in objects:
                    phasecnn = "yellow"
                    sc.speed = 30
                elif "green" in objects:
                    phasecnn = "green"
                    sc.speed = 40

                sc.lane_detection()
                sc.handle_actuators()
                sendframe(sc.frame)
                phase2send = phasecnn
                current_action = f"TL via CNN: {phasecnn}"

            elif mode == "quit":
                break

            else:  # standby o modo no definido
                sc.speed = 0
                sc.p_part = 0
                sc.i_part = 0
                sc.d_part = 0
                sc.handle_actuators()
                sendframe(sc.frame)
                current_action = "Standby Mode"

            publish_mqtt(client, mode, sc.speed, sc.steer, sc.campan, sc.camtilt, phase2send, sc.vcc)

    finally:
        sc = None
        client.publish("SMARTCAR_status/alive", "false", retain=False)
        client.loop_stop()
        client.unsubscribe(tl)
        client.unsubscribe("SMARTCAR_control/mode")
        client.unsubscribe("SMARTCAR_control/manual_action")
        client.unsubscribe("SMARTCAR_control/sound")
        client.disconnect()

if __name__ == "__main__":
    main()