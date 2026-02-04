from ultralytics import YOLO
import cv2
import depthai as dai

model = YOLO("yolo26n.pt")

pipeline = dai.Pipeline()

cam = pipeline.create(dai.node.Camera).build()
cameraOutput = cam.requestOutput((640, 360), type=dai.ImgFrame.Type.BGR888i)
outputQueue = cameraOutput.createOutputQueue()

pipeline.start()

while pipeline.isRunning():
    frame = outputQueue.get().getFrame()

    results = model(frame, verbose = False)

    annotated = results[0].plot()

    cv2.imshow("Demo", annotated)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()