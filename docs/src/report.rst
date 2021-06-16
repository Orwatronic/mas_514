Project Report
==============
The project report shall be written using the `Sphinx Documentation <https://www.sphinx-doc.org/en/master/>`_ system. This system is widely used to document and describe open-source projects by developers worldwide. Knowing to use this system will give you valuable insight in how Agile teams document their software/products in parallel with the development process.

Sphinx support both Markdown and reStructuredText to format the pages, hence a brief introduction to the two markdown languages are found here:

* `reStructuredText <https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html>`_
* `Markdown <https://markdown-guide.readthedocs.io/en/latest/basics.html>`_

To write Math, reStructuredText is preferred since it is very hard to get math working in Markdown using Sphinx unfortunately. However, reStructuredText is in the end more complete if compared to Markdown, which is why reStructuredText should be your first option to write the report using Sphinx.

Sphinx Style guide
------------------
This webpage contains a nice overview of the Sphinx styles and syntax: https://documentation-style-guide-sphinx.readthedocs.io/en/latest/index.html

Sphinx and CI/CD
-------------------------------------
To get started with the report, the easiest way to get up to speed is to simply fork the MAS514 repository on GitLab and clone it to your a folder in your computer. Follow the steps below to set up your computer:

#.  Download and install Git from: https://git-scm.com/download

    * Open Command Prompt and configure Git globally on your computer:
    * ``git config --global user.name "John Doe"``
    * ``git config --global user.email "john.doe@mail.com"``

#.  Create a GitLab user on https://gitlab.com
#.  Your Merge master visits https://gitlab.com/hagenmek/mas514 and forks the course repo from your GitLab account.
#.  Merge master invites the other group members to the mas514 repo. 
#.  All group members clones the repo to their own computers in a non-synced folder (e.g. Dropbox, Google Drive) executing: ``git clone https://gitlab.com/<merge-master-username>/mas514`` inside the destination folder e.g. ``C:\GitLab\``.
#. Open ``C:\GitLab\mas514`` folder in Visual Studio Code and try to edit the ``C:\GitLab\mas514\docs\src\ros.md`` by writing your name in it for example.
#. Commit the changes by ``git add -A`` followed by a commit ``git commit -m "Test commit from my local computer"``.
#. After the commit, the changes shall be pushed to GitLab using ``git push``
#. This will start the CI/CD Process on your GitLab account and after about a minute your very own webpage shall appear on ``https://<merge-master-username>.gitlab.io/mas514``
#. Congrats, your first webpage made with Sphinx is now online as a result of using DevOps CI/CD methods!

Please notice that the recipe for the automatic build executed by GitLab is described by the GitLab runner file ``.gitlab-ci.yml`` found the root folder of the MAS514 repo. 

Local Sphinx Testing
------------------------------------
This step assumes that you have installed `Anaconda 3 <https://www.anaconda.com/products/individual>`_ on your local computer. The following step is required to work with the documentation locally.

#. Open Anaconda Command Prompt and install the required packages using pip:

    * ``cd C:\<gitlab_folder>\mas514\docs``
    * ``pip install -r requirements.txt``

After that the Python packages have been installed, the local autobuild feature can be started by executing ``start serve.bat`` from the ``./docs/`` folder. Visit the generated webpage on http://127.0.0.1:8000. The autobuild system will detect a any changes and automatically rebuild your webpage if some of the files are changed in the ``docs`` folder. Go ahead and try it out!


Math in reST
------------
reStructuredText supports math, please visit https://www.sphinx-doc.org/en/1.0/ext/math.html for more examples.

.. math::

   (a + b)^2 = a^2 + 2ab + b^2

   (a - b)^2 = a^2 - 2ab + b^2



