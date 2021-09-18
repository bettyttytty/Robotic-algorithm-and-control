from utils import np

def Light(imgs):
    maxNum = 0
    indexMax = 0
    for index, img in enumerate(imgs):
        count = np.sum(img)
        if count > maxNum:
            maxNum = count
            indexMax = index

    return indexMax*(180/(len(imgs)-1))
            

