import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *
import serial
import time

# Seri port ayarları
useSerial = True  # Seri port kullanımı
useQuat = False   # Kuaterniyon yerine yaw, pitch, roll kullan
SERIAL_PORT = 'COM9'
BAUD_RATE = 115200
TIMEOUT = 0.1  # Seri port zaman aşımı süresi (saniye)

# Seri port bağlantısını başlat
ser = None
last_yaw, last_pitch, last_roll = 0.0, 0.0, 0.0  # Son bilinen değerler

def init_serial():
    """Seri port bağlantısını başlatır veya yeniden bağlanır."""
    global ser
    try:
        if ser is not None and ser.is_open:
            ser.close()  # Önceki bağlantıyı kapat
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        print(f"{SERIAL_PORT} portuna bağlanıldı.")
        return True
    except serial.SerialException as e:
        print(f"Seri port bağlantı hatası: {e}")
        ser = None
        return False

def read_serial():
    """Seri porttan veri okur ve yaw, pitch, roll döndürür."""
    global last_yaw, last_pitch, last_roll
    try:
        if ser is not None and ser.is_open:
            line = ser.readline().decode('UTF-8').strip()
            if line:  # Veri varsa
                values = line.split()
                if len(values) >= 3:  # Yeterli veri var mı kontrol et
                    yaw = float(values[0])
                    pitch = float(values[1])
                    roll = -float(values[2])
                    last_yaw, last_pitch, last_roll = yaw, pitch, roll
                    return yaw, pitch, roll
            return last_yaw, last_pitch, last_roll  # Veri yoksa son bilinen değerleri döndür
        else:
            return last_yaw, last_pitch, last_roll  # Bağlantı yoksa son bilinen değerleri döndür
    except (serial.SerialException, ValueError) as e:
        print(f"Veri okuma hatası: {e}")
        init_serial()  # Bağlantıyı yeniden dene
        return last_yaw, last_pitch, last_roll

def main():
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((1920, 1080), video_flags)
    pygame.display.set_caption("AHRS - MPU6050 QUATERNION")
    resizewin(1920, 1080)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()

    # Seri portu başlat
    init_serial()

    while True:
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break

        # Seri porttan veri oku
        yaw, pitch, roll = read_serial()

        # Kalibrasyon (gerekirse)
        pitch -= 0
        roll -= 0

        draw(1, yaw, pitch, roll)
        pygame.display.flip()
        frames += 1

    print("fps: %d" % ((frames*1000)/(pygame.time.get_ticks()-ticks)))
    if useSerial and ser is not None and ser.is_open:
        ser.close()

def resizewin(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def draw(w, nx, ny, nz):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    drawText((-2.6, 1.8, 2), "MPU6050 AHRS System", 18)
    drawText((-2.6, 1.6, 2), "MPU6050 AHRS Implementation - Using Quaternion based filter", 16)
    drawText((-2.6, -2, 2), "Press Escape to exit.", 16)

    yaw = nx
    pitch = ny
    roll = nz
    drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" % (yaw, pitch, roll), 16)
    glRotatef(-roll, 0.00, 0.00, 1.00)
    glRotatef(pitch, 1.00, 0.00, 0.00)
    glRotatef(yaw, 0.00, 1.00, 0.00)

    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()

def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

if __name__ == '__main__':
    main()