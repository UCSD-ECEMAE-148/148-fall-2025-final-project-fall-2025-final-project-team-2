from collections import deque
import numpy as np

CONSTANTE_DISTANCE = 10
W_PERS  = 10000 
W_TEMP  = 10     
W_NEIGH  = 0      
W_DELTA = 50   
W_DIST  = 20    

class Summit:
    def __init__(self, id, temperature, a_personne, voisins_ids):
        self.id = id
        self.temperature = temperature
        self.a_personne = a_personne
        self.voisins_ids = voisins_ids
        self.temp_precedente = temperature
        self.delta_t = 0

    def update_temperature(self, new_temp):
        self.delta_t = new_temp - self.temp_precedente
        self.temp_precedente = new_temp
        self.temperature = new_temp

def distance_bfs(depart_id, cible_id, table_sommets):
    if depart_id == cible_id:
        return 0
    file = deque([(depart_id, 0)])
    visites = {depart_id}
    map_sommets = {s.id: s for s in table_sommets}
    while file:
        id_courant, sauts = file.popleft()
        if id_courant == cible_id:
            return sauts
        sommet = map_sommets.get(id_courant)
        if not sommet: 
            continue
        for v_id in sommet.voisins_ids:
            if v_id not in visites:
                visites.add(v_id)
                file.append((v_id, sauts + 1))
    return 999 # in case there is a summit where we can't go to 

import numpy as np

def build_or_update_summits_from_matrices(temp_matrix, person_matrix, existing_summits=None):
    """
    Convert temp_matrix & person_matrix into:
      - table_sommets: list[Summit]
      - summits_by_id: dict[id -> Summit]
      - id_matrix: matrix of summit ids for each (row, col), or -1 if no window
    For now we ignore existing_summits and rebuild each frame (simpler).
    """
    if existing_summits is None:
        existing_summits = {}

    rows, cols = temp_matrix.shape
    id_matrix = -np.ones_like(temp_matrix, dtype=int)

    table_sommets = []
    next_id = 0

    # 1) Create a Summit for every valid window
    for r in range(rows):
        for c in range(cols):
            T = temp_matrix[r, c]
            if np.isnan(T):
                continue  # no window here

            has_person = bool(person_matrix[r, c])

            summit_id = next_id
            next_id += 1

            id_matrix[r, c] = summit_id

            s = Summit(
                id=summit_id,
                temperature=float(T),
                a_personne=has_person,
                voisins_ids=[]
            )
            table_sommets.append(s)

    # 2) Build neighbors from 4-connected grid (up/down/left/right)
    summits_by_id = {s.id: s for s in table_sommets}

    for r in range(rows):
        for c in range(cols):
            sid = id_matrix[r, c]
            if sid == -1:
                continue

            voisins = []
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                rr = r + dr
                cc = c + dc
                if 0 <= rr < rows and 0 <= cc < cols:
                    nid = id_matrix[rr, cc]
                    if nid != -1:
                        voisins.append(nid)

            summits_by_id[sid].voisins_ids = voisins

    return table_sommets, summits_by_id, id_matrix

def choisir_meilleure_cible(position_actuelle_id, table_sommets):
    meilleur_score = -float('inf')
    meilleure_cible = None

    # Map ID -> Summit for fast lookup
    map_sommets = {s.id: s for s in table_sommets}

    for candidat in table_sommets:
        # Ignore windows with no fire
        if candidat.temperature <= 0:
            continue

        # Sum neighbor temperatures
        temp_neighbor = 0
        for v_id in candidat.voisins_ids:
            if v_id in map_sommets:
                temp_neighbor += map_sommets[v_id].temperature

        # Urgency score
        score_urgence = (
            W_PERS * candidat.a_personne +
            W_TEMP * candidat.temperature +
            W_NEIGH * temp_neighbor +
            W_DELTA * candidat.delta_t
        )

        # Distance penalty
        nb_sauts = distance_bfs(position_actuelle_id, candidat.id, table_sommets)
        dist_reelle = nb_sauts * CONSTANTE_DISTANCE
        penalite_distance = W_DIST * dist_reelle

        final_score = score_urgence - penalite_distance
        print(f"window {candidat.id} -> score: {final_score}")

        if final_score > meilleur_score:
            meilleur_score = final_score
            meilleure_cible = candidat

    return meilleure_cible
