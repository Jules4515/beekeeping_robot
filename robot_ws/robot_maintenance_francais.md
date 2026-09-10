# Robot : points essentiels

## Alimentation

- Batterie du PC : environ 8 à 10 heures en utilisation intensive.
- Batteries moteurs : rester au-dessus de 48 V pour préserver leur durée de vie.
- Les moteurs cessent de fonctionner vers 46 V ; la charge complète est d'environ 53 V.
- Autonomie : plus de 30 à 40 heures selon l'utilisation.
- LiDAR : il consomme trop sur la batterie du PC ; l'alimenter avec la batterie moteurs.

## Ordre de démarrage

Pendant les essais, l'ordre de démarrage fiable était le suivant :

1. Débrancher le LiDAR.
2. Démarrer les microcontrôleurs Micro-ROS.
3. Rebrancher le LiDAR.

Le LiDAR pouvait empêcher le démarrage des microcontrôleurs s'il était alimenté
en premier.

## Entretien mécanique

- Vérifier et resserrer régulièrement les deux vis qui maintiennent les clavettes
  dans chaque roue dentée de courroie de direction.
- Vérifier les fixations des roues au châssis, notamment la grande roue dentée
  de courroie. Il y a environ huit vis.

## Télécommande

Rester à environ 15 à 20 m maximum du robot.

## Problème connu : roue arrière gauche

La roue arrière gauche a perdu son contrôle pendant le projet. Les drivers, la
carte ESP32 et l'optocoupleur 8N semblaient fonctionner normalement. Le
remplacement de la carte par celle du robot bras n'a pas résolu le problème.
Les commandes ROS 2 étaient valides, mais la roue ne répondait pas correctement ;
à un moment, le moteur de traction fonctionnait, mais pas le servo de direction.

L'optocoupleur 2N de la petite carte reliant l'ESP32 au driver de traction n'a
pas été testé. La panne exacte n'a pas été identifiée.

## Configuration Ackermann

Pour utiliser le mode Ackermann de secours :

- Débrancher l'alimentation des drivers de traction arrière pour libérer les roues.
- Laisser les roues arrière non motorisées et non directrices.

Pour restaurer le mode Swerve complet :

- Rebrancher l'alimentation des drivers de traction arrière.
- Remettre sur le robot la carte microcontrôleur rangée dans un sachet à côté du bras.
- Remettre la carte actuellement installée sur le robot sur le robot bras.

Le mode Ackermann a permis de poursuivre les essais malgré la panne matérielle
non résolue. Le mode Swerve complet reste la configuration prévue.
