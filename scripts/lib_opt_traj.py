from tf import TransformBroadcaster
import rospy
from rospy import Time 

def generate_cities_from_clusters(num_clust,centroids):

    #print(centroids,centroids[0][0],centroids[0][1])
    if num_clust == 0:
        print("Empty")

    if num_clust == 1:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1])
    }
        
    if num_clust == 2:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1])
    }
        
    if num_clust == 3:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1])
    }
        
    if num_clust == 4:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1])
    }
        
    if num_clust == 5:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1])
    }

    if num_clust == 6:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1])
    }
        
    if num_clust == 7:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1])
    }
        
    if num_clust == 8:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1])
    }
        
    if num_clust == 9:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1])
    }
        
    if num_clust == 10:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1])
    }
        
    if num_clust == 11:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1])
    }
        
    if num_clust == 12:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1])
    }
    
    if num_clust == 13:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1])
    }
        
    if num_clust == 14:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1])
    }
        
    if num_clust == 15:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1])
    }  

    if num_clust == 16:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1])
    }  
         
    if num_clust == 17:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1])
    }  

    if num_clust == 18:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1])
    }  
        
    if num_clust == 19:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1])
    }   
        
    if num_clust == 20:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1])
    }   
        
    if num_clust == 21:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1])
    }   
        
    if num_clust == 22:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1])
    }   

    if num_clust == 23:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1])
    }   
        
    if num_clust == 24:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1])
    }   
        
    if num_clust == 25:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1])
    }   
        
    if num_clust == 26:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1])
    }   
        
    if num_clust == 27:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1])
    }   
        
    if num_clust == 28:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1]),
        'BB': (centroids[27][0], centroids[27][1])
    }   

    if num_clust == 29:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1]),
        'BB': (centroids[27][0], centroids[27][1]),
        'CC': (centroids[28][0], centroids[28][1])
    }   
    
    if num_clust == 30:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1]),
        'BB': (centroids[27][0], centroids[27][1]),
        'CC': (centroids[28][0], centroids[28][1]),
        'DD': (centroids[29][0], centroids[29][1])
    }   

    if num_clust == 31:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1]),
        'BB': (centroids[27][0], centroids[27][1]),
        'CC': (centroids[28][0], centroids[28][1]),
        'DD': (centroids[29][0], centroids[29][1]),
        'EE': (centroids[30][0], centroids[30][1])
    }   
    
    if num_clust == 32:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1]),
        'BB': (centroids[27][0], centroids[27][1]),
        'CC': (centroids[28][0], centroids[28][1]),
        'DD': (centroids[29][0], centroids[29][1]),
        'EE': (centroids[30][0], centroids[30][1]),
        'FF': (centroids[31][0], centroids[31][1])
    }   
    
    if num_clust == 33:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1]),
        'BB': (centroids[27][0], centroids[27][1]),
        'CC': (centroids[28][0], centroids[28][1]),
        'DD': (centroids[29][0], centroids[29][1]),
        'EE': (centroids[30][0], centroids[30][1]),
        'FF': (centroids[31][0], centroids[31][1]),
        'GG': (centroids[32][0], centroids[32][1])
    }   

    if num_clust == 34:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1]),
        'BB': (centroids[27][0], centroids[27][1]),
        'CC': (centroids[28][0], centroids[28][1]),
        'DD': (centroids[29][0], centroids[29][1]),
        'EE': (centroids[30][0], centroids[30][1]),
        'FF': (centroids[31][0], centroids[31][1]),
        'GG': (centroids[32][0], centroids[32][1]),
        'HH': (centroids[33][0], centroids[33][1])
    }    

    if num_clust == 35:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1]),
        'BB': (centroids[27][0], centroids[27][1]),
        'CC': (centroids[28][0], centroids[28][1]),
        'DD': (centroids[29][0], centroids[29][1]),
        'EE': (centroids[30][0], centroids[30][1]),
        'FF': (centroids[31][0], centroids[31][1]),
        'GG': (centroids[32][0], centroids[32][1]),
        'HH': (centroids[33][0], centroids[33][1]),
        'II': (centroids[34][0], centroids[34][1])
    }    
    
    if num_clust == 36:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1]),
        'BB': (centroids[27][0], centroids[27][1]),
        'CC': (centroids[28][0], centroids[28][1]),
        'DD': (centroids[29][0], centroids[29][1]),
        'EE': (centroids[30][0], centroids[30][1]),
        'FF': (centroids[31][0], centroids[31][1]),
        'GG': (centroids[32][0], centroids[32][1]),
        'HH': (centroids[33][0], centroids[33][1]),
        'II': (centroids[34][0], centroids[34][1]),
        'JJ': (centroids[35][0], centroids[35][1])
    }    
        
    if num_clust == 37:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1]),
        'BB': (centroids[27][0], centroids[27][1]),
        'CC': (centroids[28][0], centroids[28][1]),
        'DD': (centroids[29][0], centroids[29][1]),
        'EE': (centroids[30][0], centroids[30][1]),
        'FF': (centroids[31][0], centroids[31][1]),
        'GG': (centroids[32][0], centroids[32][1]),
        'HH': (centroids[33][0], centroids[33][1]),
        'II': (centroids[34][0], centroids[34][1]),
        'JJ': (centroids[35][0], centroids[35][1]),
        'KK': (centroids[36][0], centroids[36][1])
    }    
        
    if num_clust == 38:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1]),
        'BB': (centroids[27][0], centroids[27][1]),
        'CC': (centroids[28][0], centroids[28][1]),
        'DD': (centroids[29][0], centroids[29][1]),
        'EE': (centroids[30][0], centroids[30][1]),
        'FF': (centroids[31][0], centroids[31][1]),
        'GG': (centroids[32][0], centroids[32][1]),
        'HH': (centroids[33][0], centroids[33][1]),
        'II': (centroids[34][0], centroids[34][1]),
        'JJ': (centroids[35][0], centroids[35][1]),
        'KK': (centroids[36][0], centroids[36][1]),
        'LL': (centroids[37][0], centroids[37][1])
    }    

    if num_clust == 39:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1]),
        'BB': (centroids[27][0], centroids[27][1]),
        'CC': (centroids[28][0], centroids[28][1]),
        'DD': (centroids[29][0], centroids[29][1]),
        'EE': (centroids[30][0], centroids[30][1]),
        'FF': (centroids[31][0], centroids[31][1]),
        'GG': (centroids[32][0], centroids[32][1]),
        'HH': (centroids[33][0], centroids[33][1]),
        'II': (centroids[34][0], centroids[34][1]),
        'JJ': (centroids[35][0], centroids[35][1]),
        'KK': (centroids[36][0], centroids[36][1]),
        'LL': (centroids[37][0], centroids[37][1]),
        'MM': (centroids[38][0], centroids[38][1])
    }    

    if num_clust == 40:
    
        clusters = {
        'A': (centroids[0][0], centroids[0][1]),
        'B': (centroids[1][0], centroids[1][1]),
        'C': (centroids[2][0], centroids[2][1]),
        'D': (centroids[3][0], centroids[3][1]),
        'E': (centroids[4][0], centroids[4][1]),
        'F': (centroids[5][0], centroids[5][1]),
        'G': (centroids[6][0], centroids[6][1]),
        'H': (centroids[7][0], centroids[7][1]),
        'I': (centroids[8][0], centroids[8][1]),
        'J': (centroids[9][0], centroids[9][1]),
        'K': (centroids[10][0], centroids[10][1]),
        'L': (centroids[11][0], centroids[11][1]),
        'M': (centroids[12][0], centroids[12][1]),
        'N': (centroids[13][0], centroids[13][1]),
        'O': (centroids[14][0], centroids[14][1]),
        'P': (centroids[15][0], centroids[15][1]),
        'Q': (centroids[16][0], centroids[16][1]),
        'R': (centroids[17][0], centroids[17][1]),
        'S': (centroids[18][0], centroids[18][1]),
        'T': (centroids[19][0], centroids[19][1]),
        'U': (centroids[20][0], centroids[20][1]),
        'V': (centroids[21][0], centroids[21][1]),
        'W': (centroids[22][0], centroids[22][1]),
        'X': (centroids[23][0], centroids[23][1]),
        'Y': (centroids[24][0], centroids[24][1]),
        'Z': (centroids[25][0], centroids[25][1]),
        'AA': (centroids[26][0], centroids[26][1]),
        'BB': (centroids[27][0], centroids[27][1]),
        'CC': (centroids[28][0], centroids[28][1]),
        'DD': (centroids[29][0], centroids[29][1]),
        'EE': (centroids[30][0], centroids[30][1]),
        'FF': (centroids[31][0], centroids[31][1]),
        'GG': (centroids[32][0], centroids[32][1]),
        'HH': (centroids[33][0], centroids[33][1]),
        'II': (centroids[34][0], centroids[34][1]),
        'JJ': (centroids[35][0], centroids[35][1]),
        'KK': (centroids[36][0], centroids[36][1]),
        'LL': (centroids[37][0], centroids[37][1]),
        'MM': (centroids[38][0], centroids[38][1]),
        'NN': (centroids[39][0], centroids[39][1])
    }
        
    
    print("City Generated Success...")
    return clusters
