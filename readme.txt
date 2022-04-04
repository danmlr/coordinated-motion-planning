Pour lancer le programme il suffit de lancer OptimizeMakespan avec comme argument le 
nom du fichier d'instance au format json. 
L'exploration des valeurs de p est alors réalisée automatiquement. 

Les valeurs de pmin et pmax peuvent être ajustées directement dans le code de 
OptimizeMakespan, cependant, les valeurs génériques sont suffisantes pour avoir des résultats 
satisfaisants dans la pluprat des cas. 

En sortie, on obtient la solution avec le suffixe makespan.json ainsi qu'un fichier avec le 
suffixe logHistory.csv qui permet de retracer l'historique de l'exploration par l'algorithme. 
La première colonne de ce fichier contient la valeur de p, la seconde un booléen indiquant si 
le calcul a réussi et la troisième indique le cas échéant la valeur du makespan. 

Aucune librairie externe autre que core3.jar n'est nécessaire. 