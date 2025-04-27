import cv2
import numpy as np
import matplotlib.pyplot as plt
import glob
import os

def get_calibration_params_from_txt(file_path):
    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith("P0:"):
                parts = line.strip().split()
                fx = float(parts[1])
                cx = float(parts[3])
                cy = float(parts[7])
                K = np.array([[fx, 0, cx],
                              [0,  fx, cy],
                              [0,   0,  1]])
                return K
    raise ValueError("Could not find P0 in calib.txt")

def load_images_from_folder(folder):
    images = []
    filenames = sorted(glob.glob(os.path.join(folder, '*.png')))
    for filename in filenames:
        img = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            images.append(img)
    return images

def extract_features(orb, img):
    keypoints, descriptors = orb.detectAndCompute(img, None)
    return keypoints, descriptors

def match_features(bf, des1, des2):
    matches = bf.match(des1, des2)
    matches = sorted(matches, key=lambda x: x.distance)
    return matches

def plot_trajectory(file_path):
    data = np.loadtxt(file_path)
    x, y, z = data[:, 0], data[:, 1], data[:, 2]

    plt.figure(figsize=(10, 6))
    plt.plot(x, z, 'b-', label='Camera Trajectory')
    plt.xlabel('X Position')
    plt.ylabel('Z Position')
    plt.title('Estimated Trajectory (Top-Down View)')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()
    plt.show()

def main():
    calib_file = r"E:\Storage\Academic_study\CS_AI_for_Robotics\Assignment3\data_odometry_gray\dataset\sequences\00\calib.txt"
    image_folder = r"E:\Storage\Academic_study\CS_AI_for_Robotics\Assignment3\data_odometry_gray\dataset\sequences\00\image_0"
    output_traj_file = r"E:\Storage\Academic_study\CS_AI_for_Robotics\Assignment3\trajectory.txt"

    K = get_calibration_params_from_txt(calib_file)
    images = load_images_from_folder(image_folder)
    if len(images) < 2:
        print("Not enough images to process.")
        return

    orb = cv2.ORB_create(nfeatures=1000)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    traj = np.zeros((600, 600, 3), dtype=np.uint8)
    R_f = np.eye(3)
    t_f = np.zeros((3, 1))

    # Open trajectory log file
    traj_file = open(output_traj_file, 'w')

    prev_img = images[0]
    prev_kp, prev_des = extract_features(orb, prev_img)

    for i in range(1, len(images)):
        curr_img = images[i]
        curr_kp, curr_des = extract_features(orb, curr_img)

        if prev_des is None or curr_des is None or len(prev_des) < 10 or len(curr_des) < 10:
            print(f"Skipping frame {i}: not enough descriptors.")
            prev_img = curr_img.copy()
            prev_kp, prev_des = extract_features(orb, prev_img)
            continue

        matches = match_features(bf, prev_des, curr_des)
        if len(matches) < 8:
            print(f"Skipping frame {i}: not enough matches.")
            prev_img = curr_img.copy()
            prev_kp, prev_des = extract_features(orb, prev_img)
            continue

        pts1 = np.float32([prev_kp[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
        pts2 = np.float32([curr_kp[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

        E, mask = cv2.findEssentialMat(pts2, pts1, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
        if E is None or E.shape != (3, 3):
            print(f"Skipping frame {i}: invalid essential matrix.")
            prev_img = curr_img.copy()
            prev_kp, prev_des = extract_features(orb, prev_img)
            continue

        _, R, t, inliers = cv2.recoverPose(E, pts2, pts1, K)

        scale = 1.0
        t_f += scale * R_f @ t
        R_f = R @ R_f

        x, y, z = t_f[0].item(), t_f[1].item(), t_f[2].item()
        draw_x, draw_y = int(x) + 300, int(z) + 100
        cv2.circle(traj, (draw_x, draw_y), 1, (i * 255 / len(images), 255 - i * 255 / len(images), 0), 2)

        # Write to file
        traj_file.write(f"{x:.6f} {y:.6f} {z:.6f}\n")

        # Draw matches
        img_vis = cv2.drawMatches(prev_img, prev_kp, curr_img, curr_kp, matches[:50], None,
                                  flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

        cv2.imshow("Trajectory", traj)
        cv2.imshow("Feature Matches", img_vis)

        if cv2.waitKey(1) & 0xFF == 27:
            break

        prev_img = curr_img.copy()
        prev_kp, prev_des = curr_kp, curr_des

    traj_file.close()
    cv2.destroyAllWindows()
    print(f"Trajectory saved to: {output_traj_file}")
    plot_trajectory(output_traj_file)

if __name__ == '__main__':
    main()


