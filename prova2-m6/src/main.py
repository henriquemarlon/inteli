import cv2
import datetime
from ultralytics import YOLO

input_video = cv2.VideoCapture('./assets/arsene.mp4')

if not input_video.isOpened():
    print("Erro")
    exit(1)

output_video = cv2.VideoWriter(
    "./output/output_{}.mp4".format(datetime.datetime.now().strftime("%Y%m%d%H%M%S")), cv2.VideoWriter_fourcc(*'mp4v'), 10, (int(input_video.get(cv2.CAP_PROP_FRAME_WIDTH)), int(input_video.get(cv2.CAP_PROP_FRAME_HEIGHT))))

haar_cascade = cv2.CascadeClassifier(
    filename=f"{cv2.data.haarcascades}/haarcascade_frontalface_default.xml")


while True:
    _, frame = input_video.read()

    if not _:
        break

    frame_gray = cv2.cvtColor(src=frame, code=cv2.COLOR_BGR2GRAY)

    faces = haar_cascade.detectMultiScale(
        image=frame_gray,
        scaleFactor=1.05,
        minNeighbors=20)
    x, y, w, h = faces[0]

    cv2.rectangle(
        img=frame,
        pt1=(x, y),
        pt2=(x+w, y+h),
        color=(0, 0, 255),
        thickness=3
    )

    cv2.imshow('Face detection', frame)
    output_video.write(frame)

    if cv2.waitKey(30) & 0xFF == ord('q'):
        break


output_video.release()
input_video.release()
cv2.destroyAllWindows()
