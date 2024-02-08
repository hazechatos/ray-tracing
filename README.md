version minimaliste de gkit2light, sans dependances ni openGL.

# installation 

clonez le depot, c'est tout !

`git clone https://forge.univ-lyon1.fr/JEAN-CLAUDE.IEHL/gkit3`

## générer les projets
_pourquoi ?_ gKit compile et fonctionne sur linux, windows, mac os, ios, android et meme WebGL. Chaque système dispose de plusieurs compilateurs et environnements de travail. Il n'est pas envisageable de créer et de maintenir tous ces projets manuellement. gKit utilise donc un outil : un générateur de projet, ce qui permet de décrire les projets une seule fois et c'est l'outil (premake dans ce cas...) qui génère le projet pour votre environnement de travail.

il faut donc apprendre à générer le projet pour votre environnement de travail, en utilisant premake.

sous linux, premake4 est disponible, installez le sur votre machine, si nécessaire : sudo apt install premake4
sinon, téléchargez le dans le répertoire gkit3 : cf premake5

ouvrez un terminal, et naviguez jusqu'au répertoire contenant gKit :
- windows : cherchez powershell ou windows terminal dans le menu démarrer
- linux : ctrl-alt-t,
- mac os : cherchez terminal

_rappel : commandes ls et cd pour naviguer._

### windows + codeblocks
`./premake5.exe codeblocks`

le workspace (groupe de projets) codeblocks ainsi que les projets sont crées dans le répertoire build/, ouvrez `build/gKit3.workspace`.

### windows + visual studio
pour générer une solution (groupe de projets) visual studio, il suffit de choisir la bonne version :
`./premake5.exe vs2022`

la solution visual studio ainsi que les projets sont crées dans le répertoire build/, ouvrez `build/gkit3.sln`.

### mac os + xcode
`./premake5 xcode`

### mac os + makefile
`./premake5 gmake`

le Makefile se trouve dans le répertoire de base de gKit.

### linux + makefile
`premake4 gmake` // si premake4 est installe dans le système

`premake5 gmake` // si premake5 est installe dans le système

`./premake5 gmake` // si premake5 est copie dans le repertoire de gKit

le Makefile se trouve dans le répertoire de base de gKit.

_remarque :_ si premake5 est disponible dans les paquets de votre distribution utilisez-le ! cf premake5 gmake

### linux + vscode
générez les makefiles, comme au dessus dans linux + makefile

## compilez un exemple
compilez tp1, par exemple, si vous voulez vérifiez qu'une application compile et fonctionne.

## utilisation des makefiles
les makefile peuvent générer les versions debug (cf utiliser un debugger comme gdb ou lldb) ou les versions release, plus rapide (2 ou 3 fois, interressant pour les projets avec beaucoup de calculs) :

- `make help`, affiche la liste des projets et les options disponibles,
- `make tp1`, compile la version debug de tp1,
- `make tp1 config=release`, compile la version release de tp1,
- `make tp1 config=debug`, compile la version debug de tp1,
- `make tp1 verbose=1`, compile la version debug de tp1 et affiche le détail des commandes exécutées.

les exécutables sont crées dans le répertoire `gkit3/bin`, pour les exécuter :
`bin/tp1`
