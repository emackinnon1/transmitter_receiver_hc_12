from machine import UART, Pin
import time
import network
from umqtt.simple import MQTTClient

uart = UART(1, 9600, rx=Pin(5), tx=Pin(4), bits=8, parity=None, stop=1, timeout=1)
led = machine.Pin("LED", machine.Pin.OUT)

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect("Who pooped in the pool", "Sonoffinan1!")

while not wlan.isconnected():
    print("Connecting...")
    time.sleep(1)
print("Connected!")

state_topic_msgs = {
    "1": {
        "state/pico_central_receiver/motion_sensor/": {
            "A": {
                "movement": {
                    "a": "Movement Detected",
                    "b": "No Movement"
                }
            },
            "B": {
                "battery": {},
            },
            "C": {
                "status": {
                    "a": "Online",
                    "b": "Error"
                }
            },
            "D": {
                "movement_type": {
                    "a": "Pitch",
                    "b": "Acceleration"
                }
            }
        }
    },
}

cmd_topic_msgs = {
    "cmd/pico_central_receiver/motion_sensor/movement": "1,A,a",
}

mqtt_state_move_pub_topic = "state/motion_sensor/movement"
mqtt_state_battery_pub_topic = "state/motion_sensor/battery"
mqtt_receive_topic = "cmd/motion_sensor/movement"
PING_INTERVAL = 60
mqtt_con_flag = False
pingresp_rcv_flag = True
lock = True
next_ping_time = 0

mqtt_server = '10.0.0.91'
mqtt_user = 'mqtt_user'
mqtt_password = 'Sonoffinan'
client_id = 'central_receiver_pico_w'

mqtt_client = MQTTClient(
    client_id=client_id,
    server=mqtt_server,
    user=mqtt_user,
    password=mqtt_password,
    keepalive=3600
)

def mqtt_subscription_callback(topic, message):
    print (f'Topic {topic} received message {message}')
    msg = message.decode('utf-8')
    tpc = topic.decode('utf-8')
    if cmd_topic_msgs[tpc]:
        topic, msg = parse(msg)
        mqtt_client.publish(topic, msg, retain=True)

mqtt_client.set_callback(mqtt_subscription_callback)

def mqtt_connect():
    global next_ping_time 
    global pingresp_rcv_flag
    global mqtt_con_flag
    global lock

    while not mqtt_con_flag:
        print("trying to connect to mqtt server.")
        led.off()
        try:
            mqtt_client.connect()
            for topic in cmd_topic_msgs.keys():
                print("subscribed to", topic)
                mqtt_client.subscribe(topic)
            mqtt_con_flag = True
            pingresp_rcv_flag = True
            next_ping_time = time.time() + PING_INTERVAL
            lock = False
        except Exception as e:
            print("Error in mqtt connect: [Exception]  %s: %s" % (type(e).__name__, e))
        time.sleep(0.5)

        print("Connected and subscribed to mqtt")
    led.on()

def ping_reset():
    global next_ping_time
    next_ping_time = time.time() + PING_INTERVAL
    print("Next MQTT ping at", next_ping_time)

def ping():
    mqtt_client.ping()
    ping_reset()
    
def check():
    global next_ping_time
    global mqtt_con_flag
    global pingresp_rcv_flag
    if (time.time() >= next_ping_time):
        ping()
#         if not pingresp_rcv_flag:
#             mqtt_con_flag = False
#             print("We have not received PINGRESP so broker disconnected")
#         else:
#             print("MQTT ping at", time.time())
#             ping()
#             ping_resp_rcv_flag = False
#     res = mqtt_client.check_msg()
#     if(res == b"PINGRESP"):
#         pingresp_rcv_flag = True
#         print("PINGRESP")
#     else:
#         ping()
        res = mqtt_client.check_msg()
#         if(res == b"PINGRESP"):
#             pingresp_rcv_flag = True
#             print("PINGRESP")
    mqtt_client.check_msg()
    
def parse(msg):
    split_msg = msg.split(',')
    indexes = [i for i in split_msg if i != ',']
    base_topic_index = indexes[0]
    base_topic = None
    topic_category = None
    msg_to_send = None
    topic = None
    try:
        if base_topic_index in state_topic_msgs:
            _base = list(state_topic_msgs[base_topic_index])
            base_topic = _base[0]
            topic_category_index = indexes[1]
            if topic_category_index in state_topic_msgs[base_topic_index][base_topic]:
                topic_category = list(state_topic_msgs[base_topic_index][base_topic][topic_category_index])[0]
                msg_index = indexes[2]
                if msg_index in state_topic_msgs[base_topic_index][base_topic][topic_category_index][topic_category]:
                    msg_to_send = state_topic_msgs[base_topic_index][base_topic][topic_category_index][topic_category][msg_index]
                elif msg_index != ' ':
                    msg_to_send = msg_index
        if all((base_topic, topic_category, msg_to_send)):
            topic = base_topic + topic_category
    except Exception as e:
        mqtt_client.publish("state/pico_central_receiver/", str(e))
    return topic, msg_to_send

def check_transmission():
    b = None
    msg = ""
    if uart.any():
        b = uart.read()
        print("uart.read:", b)
        try:
            msg = b.decode('utf-8')
            print("decoded msg: ", msg)
            topic, msg = parse(msg)
            if topic and msg:
                mqtt_client.publish(topic, msg, retain=True)
                print("success", topic, msg)
        except Exception as e:
            print (e)

while True:
    mqtt_connect()
    check_transmission()
    try:
        check()
    except Exception as e:
        print("Error in Mqtt check message: [Exception] %s: %s" % (type(e).__name__, e))
        lock = True
        mqtt_con_flag = False
    time.sleep(.1)