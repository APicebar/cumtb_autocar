def detect(img, base_column: int, full=False):
    cnt = 0
    for i in img[base_column]:
        if i == 0: cnt += 1
        if cnt > 55 and (not full): return True
        if cnt > 480 and full: return True
    return False