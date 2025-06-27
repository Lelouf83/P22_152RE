**Projet IKS01A3/L152RE**

##  Prérequis

Avant de commencer, assurez-vous d'avoir installé :

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- Une carte STM32  Nucleo-L152RE
- Le shield IKS01A3
- La carte ISEN
- Un câble micro-USB pour relier la carte à votre PC

---

##  Étape 1 : Importer le Projet dans STM32CubeIDE

1. **Ouvrez STM32CubeIDE**
2. Allez dans : `File` → `Import`
3. Choisissez : `Existing Projects into Workspace` → Cliquez sur `Next`
4. Sélectionnez le répertoire du projet (contenant le fichier `.project`)
5. Cochez votre projet, puis cliquez sur `Finish`

---

##  Étape 2 : Compiler le Projet

1. Dans l'explorateur de projets (Project Explorer), faites un clic droit sur le projet importé
2. Cliquez sur **Build Project** ou utilisez le raccourci `Ctrl + B`
3. Attendez que la compilation se termine (la console doit indiquer `Build Finished` sans erreurs)

---

##  Étape 3 : Connecter et Flasher la Carte

1. Branchez votre carte STM32 à l'ordinateur via le câble USB
2. Cliquez sur l’icône **Debug** (le petit insecte en haut) ou faites clic droit → `Debug As` → `STM32 Cortex-M C/C++ Application`
3. L'IDE va compiler à nouveau (si nécessaire), programmer la carte et lancer le debug
4. Une fois le flash terminé, cliquez sur **Resume (F8)** pour exécuter le programme

Vous pouvez également utiliser le bouton **Run** (flèche verte) pour lancer directement le programme sans debug.
---

##  Dépannage

- **Erreur ST-LINK non détecté** : vérifiez que le câble USB fonctionne et que les pilotes ST-LINK sont installés.
- **Erreur de compilation** : regardez les messages dans la console pour trouver les erreurs (`Console` en bas de l'écran).
- **Pas de réaction sur la carte** : vérifiez que le bon microcontrôleur est sélectionné dans le `.ioc` (CubeMX) et que les broches sont bien configurées.

---
Architecture du projet : 
ProjFinalSur152/
├── Core/ # Fichiers sources principaux (main.c, stm32xx_it.c, etc.)
├── Drivers/ # Drivers CMSIS et HAL et sensors !!!
├── .ioc # Fichier de configuration STM32CubeMX
└── project, .cproject # Fichiers d'import Eclipse


## Utilisation de l'application

Quatre mouvements sont reconnus par le capteur LIS2DW12 : 
- vertical
- tremblement "shake"
- Horizontal
- Immobile 

Si la carte est immobile alors les potentiometres permettent d'activer jusqu'a deux Led sur la carte Isen, si une certaine valeurs est atteinte alors la deuxieme LED clignote (fonction adcXblinking), moteur et buzzer desactivés.

Si un mouvement vertical est reconnue , moteur activé, buzzer desactivé.


Si un mouvement Horizontal est reconnue  moteur et buzzer activés.

Si un tremblement est reconnue buzzer activé, moteur desactivé, et clignotements des LED0 et LED1.

Le boutons BP3 permet d'allumer ou teindre le buzzer peut importe le mouvement dectecté.
De-même pour le BP2 mais cette fois ci pour le buzzer.

Le timer 6 permet d'activer l'effet anti rebonds
