import cv2 as cv
# from cv2.typing import MatLike

def err_calc(img, base_column: int, maximum: int):
    err = 0
    cnt = 0
    mid = (len(img[base_column])+1) / 2
    for value in img[base_column]:
        cnt += 1
        if value == 0:
            err += cnt - mid if abs(cnt-mid) < maximum else maximum if cnt-mid > 0 else -maximum

    
    # cv.imshow("process", img)
    return err / 1000


if __name__ == '__main__':
    cam = cv.VideoCapture(0)
    cam.read()

    while True:
        ret, frame = cam.read()
        print(err_calc(frame, 105, 400, 120))

        if cv.waitKey(1) == ord('q'): break
