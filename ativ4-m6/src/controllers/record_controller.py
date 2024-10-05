import cv2
import datetime


class RecordController:
    def __init__(self):
        self.video = cv2.VideoCapture(0)
        self.output = "./view/assets/video_{}.mp4".format(datetime.datetime.now(
        ).strftime("%Y%m%d%H%M%S"))
        self.writer = cv2.VideoWriter(self.output, 
        cv2.VideoWriter_fourcc(*'mp4v'), 
        10, 
        (int(self.video.get(3)), int(self.video.get(4)))
        )

    def record(self):

        while (True):

            _, frame = self.video.read()

            if _ == True:
                self.writer.write(frame)
                cv2.imshow('Frame', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                break

        self.video.release()
        self.writer.release()
        cv2.destroyAllWindows()
        return self.output


# rc = RecordController()

