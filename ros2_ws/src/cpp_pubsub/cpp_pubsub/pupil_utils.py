import math
import pywt
import numpy as np
from numpy import isnan


################################################################
def clean_up_list(mylist: list[float]):
    length = len(mylist)
    return_list = []
    nans = 0
    for item in mylist:
        if isnan(float(item)):
            nans += 1
        else:
            return_list.append(float(item))
    if len(return_list) == 0:
        return_list = [3.2 for i in range(nans)]
        
    return return_list, float(nans / length)*100.0


################################################################
def double_interp_list(mylist: list[float]):
    og_len = len(mylist)
    newlist = []
    for i in range(og_len-1):
        newlist.append(mylist[i])
        interp_num = (mylist[i+1] + mylist[i]) / 2
        newlist.append(interp_num)
    newlist.append(mylist[-1])
    return newlist


################################################################
def modmax(d):
    # compute signal modulus
    m = [0.0]*len(d)
    for i in range(len(d)):
        m[i] = math.fabs(d[i])
    # if value is larger than both neighbours, and strictly
    # larger than either, then it is a local maximum
    t = [0.0]*len(d)
    for i in range(len(d)):
        ll = m[i-1] if i >= 1 else m[i]
        oo = m[i]
        rr = m[i+1] if i < len(d)-2 else m[i]
        if (ll <= oo and oo >= rr) and (ll < oo or oo > rr):
            # compute magnitude
            t[i] = math.sqrt(d[i]**2)
        else:
            t[i] = 0.0
    return t


################################################################
def lhipa(d, duration, recalculate):

    # double interp list if duration shorter than 10 seconds, and double duration
    if recalculate:
        print("\n Recalculating LHIPA index now!\n")
        d = double_interp_list(d)
        duration *= 2

    # find max decomposition level
    w = pywt.Wavelet('sym16')
    maxlevel = pywt.dwt_max_level(len(d), filter_len=w.dec_len)
    # set high and low frequency band indices
    if int(maxlevel/2) < 2:
        max_level = 4
    hif, lof = 1, int(maxlevel/2)
    # hif, lof = 1, 2
    # get detail coefficients of pupil diameter signal d
    cD_H = pywt.downcoef('d', d, 'sym16', 'per', level=hif)
    cD_L = pywt.downcoef('d', d, 'sym16', 'per', level=lof)
    # normalize by 1/sqrt(2j)
    cD_H[:] = [x / math.sqrt(2**hif) for x in cD_H]
    cD_L[:] = [x / math.sqrt(2**lof) for x in cD_L]
    # obtain the LH:HF ratio
    cD_LH = cD_L
    for i in range(len(cD_L)):
        my_index = int(((2**lof)/(2**hif))*i)
        cD_LH[i] = cD_L[i] / cD_H[my_index]
    # detect modulus maxima, see Duchowski et al. [15]
    cD_LHm = modmax(cD_LH)    # this is a list
    # threshold using universal threshold
    # where sigma is the standard deviation of the noise
    lambda_univ = np.std(cD_LHm) * math.sqrt(2.0*np.log2(len(cD_LHm)))      # numpy.float64
    cD_LHt = pywt.threshold(cD_LHm, lambda_univ, mode='less')               # numpy.ndarray
    # get signal duration (in seconds)
    tt = duration
    # tt = d[-1].timestamp() - d[0].timestamp()
    # tt = 10             # hard-code 10 seconds
    # compute LHIPA
    ctr = 0
    for i in range(len(cD_LHt)):
        if math.fabs(cD_LHt[i]) > 0:
            ctr += 1
    LHIPA = float(ctr)/tt
    return LHIPA
