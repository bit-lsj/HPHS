import numpy as np
from utils import distance

def compute_distance_matrix(seq1, seq2):
    n = len(seq1)
    m = len(seq2)
    D = np.zeros((n, m))

    for i in range(n):
        for j in range(m):
            D[i, j] = distance(seq1[i], seq2[j])

    return D

def dtw(seq1, seq2):
    n = len(seq1)
    m = len(seq2)
    D = compute_distance_matrix(seq1, seq2)
    
    dp = np.zeros((n, m))
    dp[0, 0] = D[0, 0]

    for i in range(1, n):
        dp[i, 0] = dp[i-1, 0] + D[i, 0]
    
    for j in range(1, m):
        dp[0, j] = dp[0, j-1] + D[0, j]

    for i in range(1, n):
        for j in range(1, m):
            cost = D[i, j]
            dp[i, j] = cost + min(dp[i-1, j], dp[i, j-1], dp[i-1, j-1])

    return dp[-1, -1]

def test():
    # 5, 2, 1, 3, 8
    seq1 = [[5.1750003848224875, -0.025000247731805203], [5.1750003848224875, -5.925000335648656], [-0.024999692663550732, -5.925000335648656], [-5.224999770149589, -0.025000247731805203], [5.1750003848224875, 5.874999840185046]]
    # 5, 2, 1, 3, 6
    seq2 = [[5.183333718279998, -0.12499986551702102], [5.183333718279998, -6.09166662109395], [3.0770897874177194e-07, -6.09166662109395], [-5.18333310286204, -0.12499986551702102], [-5.18333310286204, 5.841666890059907]]
    score = dtw(seq1, seq2)
    print("DTW Similarity Score:", score)

if __name__ == "__main__":
    test()