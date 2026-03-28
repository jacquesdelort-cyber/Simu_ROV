\--- Create\_point\_with\_target\_length



Merci d'implémenter une fonction create\_\_point\_with\_target\_length(\_P, l\_seg\_target) -> (res, C)   où P est une liste de points, l\_seg\_target un nombre, res un booléen et C un point



Cette fonction renvoie un point C tels AC = l\_seg\_target et BC = l\_seg\_target

On note A = \_P(0) le premier point de \_P et B = \_P(-1) le dernier point de \_P. 

On note H le milieu de AB

Si U est un point de coordonnées (x, y) on note Ux son abscisse (et donc Ux == x) et Uy son ordonnée (et donc Uy == y)

Si U et V sont des points, on note UV la longueur du segment UV



On clippe tous les points Z de \_P tels que Zy > 0 en forçant, pour ces points uniquement, Zy = 0



Si AB > 2\* l\_seg\_target

&#x09;return (False, H)



Si AB == 2\* l\_seg\_target

&#x20;  	return (True, H)



Si A == B

&#x09;Soit D = A + (l\_seg\_target, 0)

&#x09;return (True, D)



Soit G le barycentre des milieux des segments de \_P.

SI G est sur la droite AB

&#x09;G = G + (0, -1)

&#x09;Si G est sur la droite AB

&#x09;	G = G + (1, 0)



Soit Q le point sur la perpendiculaire à AB passant par H, tel que Q est du même côté de AB que G et tel que AQ = l\_seg\_target

Si Qy > 0

&#x09;Q = le symétrique de Q par rapport à la droite AB



return (True, Q)



Merci de créer ensuite des jeux de test avec des rapports graphique pour tester différentes configurations de \_P avec un nombre de points allant de 2 à 6 (dont les extrémités)



\--- next\_point(\_Q, \_ls, Lseg\_total, R, num\_seg\_R, s\_R, step)



J'ai modifié ainsi le pseudo code de la fonction next\_point( ) :

Merci d'implémenter la fonction next\_point(\_Q, \_ls, Lseg\_total, R, num\_seg\_R, s\_R, step) -> (ns, R)

Où: 	\_Q est une liste de points

&#x20;  	\_ls est la liste des abscisses curvilignes des points de \_Q

&#x20;  	Lseg\_total est un flottant qui représente somme des longueurs des segments de \_Q

&#x20;  	R est le point courant. Il est situé sur un des segments de \_Q

&#x09;num\_seg\_R est un entier qui représente le numéro du segment auquel appartient R

&#x20;  	s\_R est un flottant qui représente l'abscisse curviligne du point R

&#x20;  	step est un nombre flottant qui représente un incrément d'abscisse curviligne



Cette fonction renvoie un point (ns, R) où ns est un entier et R un point.



Si s\_R + step  > L\_seg\_total

&#x09;return (none, none)  # Il n'y a pas de point T possible



On part du point R et on calcule T et ns tels que: 

T est le point situé sur un segment de \_Q à l'abscisse curviligne s\_R + step

ns est le numéro de ce segment.



Merci de créer ensuite des jeux de test avec des rapports graphique pour tester différentes configurations de \_Q avec un nombre de points allant de 2 à 12 (dont les extrémités)

Dans les graphiques, faire apparaitre les points R et T et dans les encarts indiquer les valeurs suivantes

Coordonnées de R

Numéro de segment de R

Abscisse curviligne de R

step

Coordonnées de T

Numéro de segment de T

Abscisse curviligne de T





\--- \_normalize\_cable\_segments



Merci d'implémenter une fonction  \_normalize\_cable\_segments(x\_cable, y\_cable, L\_target, bateau, rov, N\_target = None) 

Cette fonction prend en entrée les mêmes paramètres que \_normalize\_cable\_geometry( ) avec la particularité que N\_target est optionnel. 

N\_target est le nombre de segments que doit comporter le câble (x\_new, y\_new) qui est retourné.



Elle renvoie (res, x\_new, y\_new) où res est un booléen qui vaut True si le résultats est ok et False si il y a un problème.



Son pseudo code est le suivant :



\_normalize\_cable\_segments(x\_cable, y\_cable, L\_target, bateau, rov, N\_target = None)



Soit \_P la suite des points (x\_cable, y\_cable)



recoller \_P(0) avec bateau

recoller \_P(-1) avec rov

L\_straight = distance (ROV, bateau)



Si L\_straight > L\_target

&#x09;Soit new\_rov tel que new\_rov est sur la demi-droite partant de bateau et passant par rov et distance(bateau, new\_rov) == l\_straight

&#x09;\_Q = liste de N\_target points équidistants places sur le segments bateau - rob (incluant bateau et rov)

&#x09;return (False, \_Qx, \_Qy)



N\_seg = len(\_P) -1



Si N\_target == None

&#x09;N\_target= N\_seg



Si N\_target < 2 

&#x09;return (False, x\_cable, y\_cable)



L\_segments = somme des longueurs des segments de P

l\_seg = L\_segments / N\_target

l\_seg\_target = L\_target / N\_target



\_ls = liste des abscisses curvilignes des points de \_P

\_Q = liste vide





n\_curr = 0

P\_curr = \_P(0)

s\_curr = 0



rov\_atteint = False

tant que rov\_atteint = False

&#x09;\_Q.append(P\_curr)

&#x09;(ns, P\_new) = next\_point(\_P, \_ls, L\_segments, P\_curr, n\_curr, s\_curr, 2\*l\_seg\_target)

&#x09;si P\_new != P\_curr

&#x09;	# On construit la sous-liste à optimiser

&#x09;	\_Z = vide

&#x09;	\_Z(0) = P(n\_curr)

&#x09;	tant que \_ls(n\_curr+1) <= s\_curr + l\_seg\_target

&#x09;		\_Z.append(\_P(n\_curr+1))

&#x09;		n\_curr = n\_curr + 1

&#x09;	(res, H) = create\_\_point\_with\_target\_length(\_Z, l\_seg\_target)

&#x09;	\_Q.append(H)

&#x09;	\_Q.append(\_Z(-1)

&#x09;	P\_curr = P\_new

&#x09;else

&#x09;	rov\_atteint = True		

&#x09;	



L\_final = somme des longueurs de segments de \_Q

lsdg\_min = min des longueurs de segments de \_Q

lseg\_max = max des longueurs de segments de \_Q



assert len(\_Q)  == N\_target + 1

assert L\_final  == L\_target

assert lseg\_max == l\_seg\_target 

assert lseg\_min == l\_seg\_target



return (True, \_Qx, \_Qy)

&#x09;	

&#x09;	

Dans les encarts des tests, indiquer: 

Nb points de \_P

Longueur de \_P

Min longueur segments de \_P

Max longueur segments de \_P

Nb points de \_Q

Longueur de \_Q

Min longueur segments de \_Q

Max longueur segments de \_Q







&#x09;   

