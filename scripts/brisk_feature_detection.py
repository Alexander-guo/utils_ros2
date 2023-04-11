#!/usr/bin/env python

import cv2
import argparse
import os
from tqdm import tqdm

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="brisk_feature_detection",
        description="Detect and draw brisk features",
        add_help=True,
    )
    parser.add_argument(
        "--image", "-i", type=str, help="path to image for feature detection"
    )
    parser.add_argument(
        "--folder", "-f", type=str, help="path to folder containing images"
    )
    parser.add_argument("--display", "-d", action="store_true", help="display images")

    args = parser.parse_args()
    folder = args.folder
    image = args.image
    display_images = args.display

    brisk_detector = cv2.BRISK_create(thresh=20, octaves=1)
    if folder is not None:
        feature_folder = os.path.join(folder, "features")
        if not os.path.exists(feature_folder):
            os.makedirs(feature_folder)

        for filename in tqdm(sorted(os.listdir(folder))):
            f = os.path.join(folder, filename)
            img = cv2.imread(f, cv2.IMREAD_COLOR)
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            kps = brisk_detector.detect(img_gray, None)
            cv2.drawKeypoints(
                img,
                kps,
                img,
                color=(0, 0, 255),
                flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
            )
            if display_images:
                cv2.imshow("image", img)
                cv2.waitKey(1)
            cv2.imwrite(os.path.join(feature_folder, filename), img)

    elif image is not None:
        img = cv2.imread(image, cv2.IMREAD_COLOR)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kps = brisk_detector.detect(img_gray, None)
        cv2.drawKeypoints(
            img,
            kps,
            img,
            color=(0, 0, 255),
            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )
        if display_images:
            cv2.imshow("image", img)
            cv2.waitKey(0)
    else:
        print("Please provide an image or a folder")
