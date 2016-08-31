#Using Git
Git usually comes built-in with the main OS's. 
If it is not install or a newer version is needed, it can be downloaded from https://git-scm.com/downloads

This guide was tested in Linux.

## Cloning the repository
1. Open Terminal
2. cd to where the repository will be downloaded
3. git clone https://github.com/EagleKnights/SDI-11911.git --single-branch
4. cd into the downloaded repository

## Branches
Git uses branches to keep different versions of the code in the same repository.
The main branch is call "master" and in this repository only administrator will be able to make changes to it. After making git clone, the main branch will be "master".
To make a new branch, be sure to be inside the repo and type: 
```
git branch *new_branch*
```
This only creates tha branch, to switch into the new branch:
```
git checkoout *name_of_the_branch*
```

In this new branch is where changes can be made and push to github.

## Add - Commit - Push
The basic workflow to save changes into git is:
```
git add *files_to_add*
git commit -m "Comments about this comit"
```
After executing git commit, the changes are store

NOTE: if the option -m is not use, git will open VIM to enter the comment. A VIM cheat-sheet can be found in http://vim.rtorr.com/ .

After executing commit, the changes are store locally. To store the changes to the repo at github, use:
```
git push origin *branch_to_push*
```

## Pull
If there are new changes to the main github repo (called origin), use the following command (be sure to save your last changes):
```
git pull
```

## Other Stuff
* To print commit history
```
git log
```

* Graphically show the branches
```
git log --oneline --decorate --graph --all
```

* Manual of git
```
man git
```

* Pro Git Book Available at: https://git-scm.com/book/en/v2


