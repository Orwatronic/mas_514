#########
Git Usage
#########

***********************
Resolve Merge Conflicts
***********************
A merge conflict occurs when two files are merged and Git is not able to automatically merge the two files and their content. In these cases, we have to take action and resolve the conflicts manually. A merge conflict is indicated using a mark :code:`c` in VS Code's file tree as it is indicated inside the red box.

.. figure:: ../figs/git/merge-conflict.png

To resolve the merge conflicts, the files marked with a :code:`c` is simply opened and the conflict is presented in a nice manner in VS Code.

.. figure:: ../figs/git/merge-resolve-vscode.png

Here we can easily choose which of the conflicted versions we want. Either we choose to accept the "Current Change", "Incoming Change" or "Both Changes". Where the green and blue highlighted text represents the "Current Change" and the "Incoming Change" respectively. Please note that a file might have several conflicts and hence all these are highlighted as depicted above. Scroll trough the document and resolve all conflicts before saving the file. After all the conflicted files are resolved, the changes are ready to be staged/added and then committed. Finally back on track!

