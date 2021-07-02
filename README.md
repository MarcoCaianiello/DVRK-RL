# DVRK-RL
## Coppelia
La scena di coppelia da lanciare è DVRK-RL. La sfera (suture_point) che interseca il piano della ferita rappresenta il goal.

## Codice
Il codice tip_position.cpp ha lo scopo di mettere in comunicazione il simulatore, l'algoritmo (scritto in python) e il codice per la forward e backword kinematics (scritto in c++). La traiettoria viene generata in pub_trajectory.py a partire dai valori startPosition, startEuler, goalPosition e goalEuler presenti nel file ddpg_param.py. 
L'intera sequenza di punti viene pubblicata su un topic letto da tip_position.cpp in cui nella funzione execute_path viene calcolata la cinematica inversa ed inviati i valori dei giunti al simulatore.

## Demostration Buffer
Il Demostration Buffer (DB) deve avere la seguente struttura (s_t, a_t, r_t, s_t+1) dove con s_t indichiamo lo stato all'istante t, con a_t l'azione all'instante t e con r_t il reward all'istante t. Quest'ultimo deve essere generato a partire da un esecuzione ottimale del task, cioè fare in modo che l'ago approcci il suture_point con la punta perpendicolare al piano della ferita.
 
