import cv2
import numpy as np

board = np.zeros((900, 1200), dtype=np.uint8)
square = 100
for r in range(9):
    for c in range(12):
        if (r + c) % 2 == 0:
            board[r*square:(r+1)*square, c*square:(c+1)*square] = 255

cv2.imwrite('checkerboard.png', board)
print("Saved checkerboard.png")

