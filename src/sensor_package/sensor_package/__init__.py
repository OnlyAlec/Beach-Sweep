import serial
import time

def leer_distancia(serial_port):
    while True:
        if serial_port.in_waiting >= 9:
            data = serial_port.read(9)
            if data[0] == 0x59 and data[1] == 0x59:
                checksum = sum(data[0:8]) & 0xFF
                if checksum == data[8]:
                    distancia = data[2] + data[3] * 256  # en milímetros
                    distancia_metros = distancia / 1000.0
                    return distancia_metros

def main():
    try:
        # Cambia el puerto según tu sistema (ej: "COM3" en Windows, "/dev/ttyUSB0" en Linux)
        puerto = serial.Serial("/dev/serial0", 115200, timeout=1)  # Asegúrate de que el puerto sea correcto
        time.sleep(2)  # espera para que el sensor se estabilice

        while True:
            distancia = leer_distancia(puerto)
            print(f"Distancia: {distancia:.2f} m")
            time.sleep(0.1)  # Espera un poco para no saturar el puerto

    except KeyboardInterrupt:
        print("Programa detenido.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
