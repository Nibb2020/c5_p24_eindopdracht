#!/usr/bin/env python3  # Gebruik Python 3 als interpreter

# =========================================================
# Imports
# =========================================================

import cv2  # Importeer OpenCV voor ArUco, ChArUco en calibratie
import depthai as dai  # Importeer DepthAI voor OAK-D camerastream
import numpy as np  # Importeer NumPy voor matrices en arrays
import pickle  # Importeer pickle voor opslaan van calibratiedata
from pathlib import Path  # Importeer Path voor nette bestandspaden

# =========================================================
# Configuration
# =========================================================

FRAME_WIDTH = 640  # Gebruik exact dezelfde breedte als vision_node.py RGB_WIDTH
FRAME_HEIGHT = 480  # Gebruik exact dezelfde hoogte als vision_node.py RGB_HEIGHT

CHARUCO_SQUARES_X = 7  # Aantal ChArUco-vakken horizontaal, dus squares, niet interne corners
CHARUCO_SQUARES_Y = 5  # Aantal ChArUco-vakken verticaal, dus squares, niet interne corners

SQUARE_SIZE_M = 0.03742  # ChArUco-vakgrootte in meters, 37.42 mm
MARKER_SIZE_M = 0.03028  # ArUco-markerzijde in meters, 30.28 mm

MIN_CHARUCO_CORNERS = 8  # Minimum aantal ChArUco-corners voor één geldige capture
MIN_CAPTURES = 40  # Minimum aantal captures voor calibratie

SAVE_FILE = Path("camera_calibration_640x480_charuco.pkl")  # Picklebestand voor calibratieresultaat

ARUCO_DICTIONARY_ID = cv2.aruco.DICT_4X4_50  # Dictionary moet exact overeenkomen met het geprinte board

PRINT_MARKER_IDS = True  # Print gevonden marker-ID's voor debuggen
PRINT_EVERY_FRAME = False  # Zet True als je elke frame marker/corner-info in terminal wilt zien

# =========================================================
# Helper Functions
# =========================================================

def create_detector_parameters():  # Maak detectorparameters voor verschillende OpenCV-versies
    if hasattr(cv2.aruco, "DetectorParameters_create"):  # Controleer oude OpenCV API
        return cv2.aruco.DetectorParameters_create()  # Maak detectorparameters via oude API
    return cv2.aruco.DetectorParameters()  # Maak detectorparameters via nieuwe API

def create_charuco_board(dictionary):  # Maak ChArUco-board voor verschillende OpenCV-versies
    if hasattr(cv2.aruco, "CharucoBoard_create"):  # Controleer oude OpenCV API
        return cv2.aruco.CharucoBoard_create(  # Maak ChArUco-board via oude API
            CHARUCO_SQUARES_X,  # Geef aantal squares in x-richting mee
            CHARUCO_SQUARES_Y,  # Geef aantal squares in y-richting mee
            SQUARE_SIZE_M,  # Geef square size in meters mee
            MARKER_SIZE_M,  # Geef marker size in meters mee
            dictionary  # Geef ArUco dictionary mee
        )
    return cv2.aruco.CharucoBoard(  # Maak ChArUco-board via nieuwe API
        (CHARUCO_SQUARES_X, CHARUCO_SQUARES_Y),  # Geef aantal squares als tuple mee
        SQUARE_SIZE_M,  # Geef square size in meters mee
        MARKER_SIZE_M,  # Geef marker size in meters mee
        dictionary  # Geef ArUco dictionary mee
    )

def print_config_output(camera_matrix, dist_coeffs, reprojection_error):  # Print config.py-ready calibratiewaarden
    print()  # Print lege regel
    print("# =========================================================")  # Print scheidingslijn
    print("# Copy this into vision/config/config.py")  # Print instructieregel
    print("# =========================================================")  # Print scheidingslijn
    print()  # Print lege regel

    print("USE_DEVICE_CALIBRATION = False  # Gebruik hardcoded ChArUco-calibratie in plaats van OAK-D EEPROM")  # Print noodzakelijke configregel
    print(f"CAMERA_CALIBRATION_WIDTH = {FRAME_WIDTH}  # Calibratiebeeldbreedte")  # Print calibratiebreedte
    print(f"CAMERA_CALIBRATION_HEIGHT = {FRAME_HEIGHT}  # Calibratiebeeldhoogte")  # Print calibratiehoogte
    print(f"CAMERA_REPROJECTION_ERROR = {float(reprojection_error):.6f}  # Gemiddelde reprojection error")  # Print reprojection error
    print()  # Print lege regel

    print("CAMERA_MATRIX = np.array([")  # Start camera matrix output
    for row in camera_matrix:  # Loop door alle matrixrijen
        print(f"    [{row[0]:.8f}, {row[1]:.8f}, {row[2]:.8f}],")  # Print één matrixrij
    print("], dtype=np.float32)")  # Sluit camera matrix output af
    print()  # Print lege regel

    flat_dist = dist_coeffs.reshape(-1)  # Maak distortioncoëfficiënten eendimensionaal
    print("DIST_COEFFS = np.array([")  # Start distortion output
    print("    " + ", ".join(f"{value:.10f}" for value in flat_dist))  # Print distortionwaarden
    print("], dtype=np.float32)")  # Sluit distortion output af
    print()  # Print lege regel

def save_calibration(camera_matrix, dist_coeffs, reprojection_error):  # Sla calibratie op als pickle
    data = {  # Maak dictionary met alle calibratiedata
        "frame_width": FRAME_WIDTH,  # Bewaar framebreedte
        "frame_height": FRAME_HEIGHT,  # Bewaar framehoogte
        "camera_matrix": camera_matrix,  # Bewaar camera matrix
        "dist_coeffs": dist_coeffs,  # Bewaar distortioncoëfficiënten
        "reprojection_error": reprojection_error,  # Bewaar reprojection error
        "charuco_squares_x": CHARUCO_SQUARES_X,  # Bewaar ChArUco squares x
        "charuco_squares_y": CHARUCO_SQUARES_Y,  # Bewaar ChArUco squares y
        "square_size_m": SQUARE_SIZE_M,  # Bewaar square size
        "marker_size_m": MARKER_SIZE_M,  # Bewaar marker size
        "aruco_dictionary_id": ARUCO_DICTIONARY_ID,  # Bewaar dictionary ID
    }

    with SAVE_FILE.open("wb") as file:  # Open picklebestand voor schrijven
        pickle.dump(data, file)  # Schrijf calibratiedata weg

    print(f"Saved calibration to: {SAVE_FILE.resolve()}")  # Print volledige opslaglocatie

def print_board_debug():  # Print boardinformatie voor controle
    max_corners = (CHARUCO_SQUARES_X - 1) * (CHARUCO_SQUARES_Y - 1)  # Bereken maximaal aantal ChArUco-corners
    marker_ratio = MARKER_SIZE_M / SQUARE_SIZE_M  # Bereken marker/square verhouding
    border_m = (SQUARE_SIZE_M - MARKER_SIZE_M) / 2.0  # Bereken witte rand per zijde
    print()  # Print lege regel
    print("Board debug:")  # Print kop
    print(f"  squares: {CHARUCO_SQUARES_X} x {CHARUCO_SQUARES_Y}")  # Print aantal squares
    print(f"  max ChArUco corners: {max_corners}")  # Print maximaal aantal ChArUco-corners
    print(f"  square size: {SQUARE_SIZE_M * 1000:.2f} mm")  # Print square size in mm
    print(f"  marker size: {MARKER_SIZE_M * 1000:.2f} mm")  # Print marker size in mm
    print(f"  marker/square ratio: {marker_ratio:.3f}")  # Print marker/square verhouding
    print(f"  white border per side: {border_m * 1000:.2f} mm")  # Print witte rand per zijde
    print()  # Print lege regel

# =========================================================
# Main Program
# =========================================================

def main():  # Definieer hoofdprogramma
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARY_ID)  # Laad de gekozen ArUco dictionary
    detector_params = create_detector_parameters()  # Maak ArUco detectorparameters
    board = create_charuco_board(dictionary)  # Maak ChArUco-boardobject

    all_charuco_corners = []  # Lijst voor alle gecapte ChArUco-corners
    all_charuco_ids = []  # Lijst voor alle gecapte ChArUco-ID's

    pipeline = dai.Pipeline()  # Maak DepthAI pipeline aan

    cam = pipeline.createColorCamera()  # Maak RGB-camera node aan
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)  # Selecteer RGB-camera CAM_A
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)  # Gebruik exact dezelfde sensorresolutie als vision_node.py
    cam.setInterleaved(False)  # Gebruik planar output zoals vision_node.py
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)  # Gebruik BGR zoals vision_node.py
    cam.setPreviewSize(FRAME_WIDTH, FRAME_HEIGHT)  # Gebruik exact dezelfde previewgrootte als vision_node.py
    cam.setPreviewKeepAspectRatio(True)  # Gebruik exact dezelfde aspect-ratio-instelling als vision_node.py

    xout = pipeline.createXLinkOut()  # Maak XLink outputstream aan
    xout.setStreamName("rgb")  # Geef outputstream de naam rgb
    cam.preview.link(xout.input)  # Link exact dezelfde previewstream als vision_node.py

    print()  # Print lege regel
    print("ChArUco calibration for OAK-D")  # Print programmatitel
    print(f"DepthAI stream: THE_4_K preview {FRAME_WIDTH}x{FRAME_HEIGHT}, keepAspectRatio=True")  # Print streaminstelling
    print(f"Board: {CHARUCO_SQUARES_X}x{CHARUCO_SQUARES_Y} squares")  # Print boardformaat
    print(f"Square size: {SQUARE_SIZE_M * 1000:.2f} mm")  # Print square size in mm
    print(f"Marker size: {MARKER_SIZE_M * 1000:.2f} mm")  # Print marker size in mm
    print_board_debug()  # Print boarddebuginformatie
    print("Controls:")  # Print controls kop
    print("  c = capture current valid ChArUco frame")  # Print capturecontrol
    print("  q = calibrate and quit")  # Print calibratiecontrol
    print("  ESC = quit without calibration")  # Print abortcontrol
    print()  # Print lege regel

    with dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH) as device:  # Open OAK-D device met USB HIGH
        queue = device.getOutputQueue("rgb", maxSize=4, blocking=False)  # Maak RGB outputqueue

        while True:  # Start live capture-loop
            packet = queue.get()  # Wacht op nieuw RGB-frame
            frame = packet.getCvFrame()  # Haal OpenCV-frame op
            display = frame.copy()  # Maak displaykopie
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Zet frame om naar grayscale

            marker_corners, marker_ids, rejected = cv2.aruco.detectMarkers(  # Detecteer ArUco-markers
                gray,  # Gebruik grayscale beeld
                dictionary,  # Gebruik gekozen dictionary
                parameters=detector_params  # Gebruik detectorparameters
            )

            marker_count = 0 if marker_ids is None else len(marker_ids)  # Bepaal aantal gevonden markers
            charuco_corners = None  # Initialiseer ChArUco-corners als None
            charuco_ids = None  # Initialiseer ChArUco-ID's als None
            charuco_count = 0  # Initialiseer aantal ChArUco-corners

            if marker_ids is not None and marker_count > 0:  # Controleer of markers gevonden zijn
                cv2.aruco.drawDetectedMarkers(display, marker_corners, marker_ids)  # Teken gevonden markers

                retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(  # Interpoleer ChArUco-corners
                    marker_corners,  # Geef markerhoeken mee
                    marker_ids,  # Geef marker-ID's mee
                    gray,  # Geef grayscale beeld mee
                    board  # Geef ChArUco-board mee
                )

                charuco_count = int(retval) if retval is not None else 0  # Zet retval naar corneraantal

                if charuco_corners is not None and charuco_ids is not None and charuco_count > 0:  # Controleer geldige ChArUco output
                    cv2.aruco.drawDetectedCornersCharuco(display, charuco_corners, charuco_ids)  # Teken ChArUco-corners

            if PRINT_EVERY_FRAME:  # Controleer of per-frame debug aan staat
                if marker_ids is not None:  # Controleer of marker-ID's bestaan
                    print(f"markers={marker_count}, charuco_corners={charuco_count}, ids={marker_ids.flatten().tolist()}")  # Print debug met IDs
                else:  # Geen markers gevonden
                    print("markers=0, charuco_corners=0")  # Print lege detectie

            status_color = (0, 255, 0) if charuco_count >= MIN_CHARUCO_CORNERS else (0, 0, 255)  # Kies groen bij voldoende corners

            cv2.putText(  # Teken statusregel
                display,  # Gebruik displaybeeld
                f"captures={len(all_charuco_corners)} markers={marker_count} charuco_corners={charuco_count}",  # Maak statustekst
                (20, 30),  # Plaats tekst linksboven
                cv2.FONT_HERSHEY_SIMPLEX,  # Gebruik OpenCV font
                0.65,  # Zet tekstgrootte
                status_color,  # Gebruik statuskleur
                2  # Gebruik lijndikte
            )

            cv2.putText(  # Teken pipeline-informatie
                display,  # Gebruik displaybeeld
                "THE_4_K preview 640x480 keepAspectRatio=True",  # Toon streamketen
                (20, 60),  # Plaats tweede regel
                cv2.FONT_HERSHEY_SIMPLEX,  # Gebruik OpenCV font
                0.55,  # Zet kleinere tekst
                (255, 255, 255),  # Gebruik witte tekst
                2  # Gebruik lijndikte
            )

            cv2.imshow("OAK-D ChArUco Calibration", display)  # Toon calibratievenster
            key = cv2.waitKey(1) & 0xFF  # Lees toetsenbordinput

            if key == ord("c"):  # Controleer of gebruiker capture wil maken
                if charuco_corners is not None and charuco_ids is not None and charuco_count >= MIN_CHARUCO_CORNERS:  # Controleer geldige capture
                    all_charuco_corners.append(charuco_corners)  # Bewaar ChArUco-corners
                    all_charuco_ids.append(charuco_ids)  # Bewaar ChArUco-ID's
                    print(f"Captured frame {len(all_charuco_corners)} with {charuco_count} ChArUco corners and {marker_count} markers")  # Print capturestatus
                    if PRINT_MARKER_IDS and marker_ids is not None:  # Controleer of marker-ID debug aan staat
                        print(f"Detected marker IDs: {marker_ids.flatten().tolist()}")  # Print gevonden marker-ID's
                else:  # Capture is niet geldig
                    print(f"Rejected frame: markers={marker_count}, charuco_corners={charuco_count}")  # Print rejectstatus
                    if PRINT_MARKER_IDS and marker_ids is not None:  # Controleer marker-ID debug
                        print(f"Detected marker IDs: {marker_ids.flatten().tolist()}")  # Print gevonden marker-ID's

            elif key == ord("q"):  # Controleer of gebruiker wil calibreren en stoppen
                break  # Stop capture-loop

            elif key == 27:  # Controleer ESC
                print("Calibration aborted")  # Print abortmelding
                cv2.destroyAllWindows()  # Sluit vensters
                return  # Stop zonder calibratie

    cv2.destroyAllWindows()  # Sluit OpenCV-vensters

    if len(all_charuco_corners) < MIN_CAPTURES:  # Controleer of genoeg captures zijn gemaakt
        print(f"Not enough captures: {len(all_charuco_corners)} captured, minimum is {MIN_CAPTURES}")  # Print foutmelding
        return  # Stop zonder calibratie

    print()  # Print lege regel
    print("Calculating ChArUco calibration...")  # Print calibratiestatus
    print()  # Print lege regel

    reprojection_error, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(  # Voer ChArUco-calibratie uit
        charucoCorners=all_charuco_corners,  # Geef alle ChArUco-corners mee
        charucoIds=all_charuco_ids,  # Geef alle ChArUco-ID's mee
        board=board,  # Geef boarddefinitie mee
        imageSize=(FRAME_WIDTH, FRAME_HEIGHT),  # Gebruik exact 640x480 als calibratiebeeldformaat
        cameraMatrix=None,  # Laat OpenCV camera matrix bepalen
        distCoeffs=None  # Laat OpenCV distortion bepalen
    )

    print("Calibration complete")  # Print succesmelding
    print(f"Reprojection error: {reprojection_error:.6f}")  # Print reprojection error
    print()  # Print lege regel
    print("Camera Matrix:")  # Print ruwe camera matrix
    print(camera_matrix)  # Print camera matrix
    print()  # Print lege regel
    print("Distortion Coefficients:")  # Print ruwe distortioncoëfficiënten
    print(dist_coeffs)  # Print distortioncoëfficiënten
    print()  # Print lege regel

    save_calibration(camera_matrix, dist_coeffs, reprojection_error)  # Sla calibratie op
    print_config_output(camera_matrix, dist_coeffs, reprojection_error)  # Print config.py-ready output

# =========================================================
# Entry Point
# =========================================================

if __name__ == "__main__":  # Controleer of script direct wordt uitgevoerd
    main()  # Start hoofdprogramma