# -*- coding: UTF-8 -*-

import dlib,numpy 
import cv2          
import time


predictor_path = "/root/Desktop/face/resource/shape_predictor_68_face_landmarks.dat"

face_rec_model_path = "/root/Desktop/face/resource/dlib_face_recognition_resnet_model_v1.dat"

candidate_npydata_path = "/root/Desktop/face/resource/candidates.npy"
candidate_path = "/root/Desktop/face/resource/candidates.txt"

path_screenshots = "/root/Desktop/face/resource/screenShots/"


def main():
    detector = dlib.get_frontal_face_detector()

    sp = dlib.shape_predictor(predictor_path)

    facerec = dlib.face_recognition_model_v1(face_rec_model_path)



    npy_data=numpy.load(candidate_npydata_path)
    descriptors=npy_data.tolist()

    candidate = []
    file=open(candidate_path, 'r')
    list_read = file.readlines()
    for name in list_read:
        name = name.strip('\n')
        candidate.append(name)


    cv2.namedWindow("camera", 1)
    cap = cv2.VideoCapture(0)
    cap.set(3, 480)

    cnt = 0
    while (cap.isOpened()):  
        ret, img = cap.read()  
        if ret == True:       

            cv2.putText(img, "press 'S': screenshot", (20, 400), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(img, "press 'Q': quit", (20, 450), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 1, cv2.LINE_AA)


            dets = detector(img, 1)
            if len(dets) != 0:
                for k, d in enumerate(dets):
                    shape = sp(img, d)
                    for pt in shape.parts():
                        pt_pos = (pt.x, pt.y)
                        cv2.circle(img, pt_pos, 2, (0, 255, 0), 1)
                    face_descriptor = facerec.compute_face_descriptor(img, shape)
                    d_test2 = numpy.array(face_descriptor)
                    dist = []
                    for i in descriptors:
                        dist_ = numpy.linalg.norm(i - d_test2)
                        dist.append(dist_)
                    num = dist.index(min(dist))

                    left_top = (dlib.rectangle.left(d), dlib.rectangle.top(d))
                    right_bottom = (dlib.rectangle.right(d), dlib.rectangle.bottom(d))
                    cv2.rectangle(img, left_top, right_bottom, (0, 255, 0), 2, cv2.LINE_AA)
                    text_point = (dlib.rectangle.left(d), dlib.rectangle.top(d) - 5)
                    cv2.putText(img, candidate[num][0:4], text_point, cv2.FONT_HERSHEY_PLAIN, 2.0, (255, 255, 255), 1, 1)  # 标出face

                cv2.putText(img, "facesNum: " + str(len(dets)), (20, 50),  cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), 2, cv2.LINE_AA)
            else:

                cv2.putText(img, "facesNum:0", (20, 50),  cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 0, 0), 2, cv2.LINE_AA)

            k = cv2.waitKey(1)

            if k == ord('s'):
                cnt += 1
                print(path_screenshots + "screenshot" + "_" + str(cnt) + "_" + time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime()) + ".jpg")
                cv2.imwrite(path_screenshots + "screenshot" + "_" + str(cnt) + "_" + time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime()) + ".jpg", img)


            if k == ord('q'):
                break
            cv2.imshow("camera", img)


    cap.release()
    cv2.destroyAllWindows()
    return
