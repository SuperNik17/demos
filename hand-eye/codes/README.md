# 📂 Codes – Hand–Eye Calibration Pipeline

This folder contains all the MATLAB scripts needed to perform the complete **hand–eye calibration** procedure for the robotic setup based on Dobot Magician and Intel RealSense D415.  
Each script corresponds to a well-defined step of the pipeline.

---

## 📋 Workflow Overview

| Step | Script | Purpose |
|:----:|:-------|:--------|
| 1️⃣ | `1captureCalibrationPoseClouds.m` | Move the robot to predefined poses, acquire 10 RGB-D frames per pose, and save the raw point clouds |
| 2️⃣ | `2averagePointClouds.m` | Average the captured frames at each pose to reduce noise and generate a single denoised point cloud per pose |
| 3️⃣ | `3extractMarkerFromRGB.m` | Perform color-based segmentation to isolate the red marker and extract 3D points belonging to the marker |
| 4️⃣ | `4extractCenterByFitting.m` | Align experimental marker clouds with a CAD model via plane fitting and ICP registration; estimate marker centers |
| 5️⃣ | `5optimizeTransformationMatrix.m` | Estimate the rigid-body transformation (rotation + translation) between camera frame and robot frame using least squares optimization |
| 6️⃣ | `computeResidualNorm.m` | Calculate the residuals between estimated and actual marker positions and evaluate the calibration accuracy |

---

## 📜 Description of Each Script

### 1️⃣ captureCalibrationPoseClouds.m
- Moves the robot through 9 predefined poses.
- Captures 10 RGB-D frames per pose.
- Saves the 3D point clouds (`I`) and the corresponding robot poses (`Ppose`).

### 2️⃣ averagePointClouds.m
- Loads the saved point clouds.
- Computes the mean of X, Y, Z coordinates across the 10 frames for each pose.
- Creates denoised point clouds for each calibration pose.

### 3️⃣ extractMarkerFromRGB.m
- Enhances the local contrast of the color images.
- Performs red-channel extraction, grayscale subtraction, sharpening, and binarization.
- Segments the red marker and extracts its corresponding 3D points.

### 4️⃣ extractCenterByFitting.m
- Builds an artificial marker model (planar with rounded corners).
- Estimates a common plane from the experimental clouds.
- Projects experimental points onto the plane and aligns them with the model using ICP.
- Calculates marker centers both geometrically (via ICP) and statistically (mean position).

### 5️⃣ optimizeTransformationMatrix.m
- Sets up a least-squares problem to estimate the transformation matrix between camera and robot frames.
- Optimizes 6 parameters: 3 rotations (Euler angles) + 3 translations.
- Outputs the optimized homogeneous transformation matrix `tform_eval`.

### 6️⃣ computeResidualNorm.m
- Applies the estimated transformation to the camera-detected centers.
- Compares the transformed points to the true robot poses.
- Computes per-axis residuals and the overall residual norm to evaluate calibration accuracy.

---

## ⚙️ Notes on Parameters

Some parameters in the scripts may need tuning depending on experimental conditions:
- Number of neighbors in `pcdenoise()` (outlier removal).
- Max iterations and tolerances in `pcregistericp()`.
- Size thresholds for binarization and morphology in marker segmentation.
- Initial guesses and bounds in optimization setup.

Please review comments inside each script for details on tunable parameters.

---

## 📂 Output Files

During execution, the following intermediate `.mat` files are generated:

| File | Content |
|:-----|:--------|
| `10pc_9positions.mat` | Raw point clouds captured at 9 poses |
| `Ppose.mat` | Robot poses corresponding to each point cloud |
| `averagedPC.mat` | Averaged point clouds (denoised) |
| `averaged_pcmarker.mat` | Extracted marker point clouds |
| `centro_fit.mat` | Estimated marker centers via fitting |
| `TMatrix9.mat` | Final camera-to-robot transformation matrix |
| `residual.mat` *(optional)* | Residuals between transformed and real points |

---

## 🧠 Authors

Developed by **Armando Nicolella**, **Chiara Cosenza**, and **Sergio Savino**  
at **LAM4R – University of Naples Federico II**.

---

## 📜 License

This code is distributed for academic and research purposes.  
Please cite the related publication if used:

> **Cosenza, C., Malfi, P., Nicolella, A., Niola, V., Savino, S.**  
> *Hand-Eye Calibration Using Invariant Calibrator Placed to a Robotic Arm*.  
> Advances in Italian Mechanism Science, Springer, 2024.  
> [DOI: 10.1007/978-3-031-64553-2_40](https://doi.org/10.1007/978-3-031-64553-2_40)

📚 **BibTeX entry:**

```bibtex
@InProceedings{Cosenza2024HandEye,
  author    = {Chiara Cosenza and Pierangelo Malfi and Armando Nicolella and Vincenzo Niola and Sergio Savino},
  editor    = {Giuseppe Quaglia and Giovanni Boschetti and Giuseppe Carbone},
  title     = {Hand-Eye Calibration Using Invariant Calibrator Placed to a Robotic Arm},
  booktitle = {Advances in Italian Mechanism Science},
  year      = {2024},
  publisher = {Springer Nature Switzerland},
  address   = {Cham},
  pages     = {343--350},
  isbn      = {978-3-031-64553-2},
  doi       = {10.1007/978-3-031-64553-2_40}
}

---


