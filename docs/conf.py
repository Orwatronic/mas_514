# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import re
# import sys
# sys.path.insert(0, os.path.abspath('.'))



# -- Project information -----------------------------------------------------

project = 'MAS507'
copyright = '2020, University of Agder'
author = 'Andreas Klausen and Sondre Sanden Tørdal'

# The full version, including alpha/beta/rc tags
version = re.sub('^v', '', os.popen('git describe').read().strip())

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'recommonmark',
    'sphinx_rtd_theme',
    'sphinxcontrib.bibtex'
]

# Handle both ReST and Markdown files
source_suffix = {
    '.rst': 'restructuredtext',
    '.txt': 'markdown',
    '.md': 'markdown'
}

# Additional variables for HTML context (used in _templates/*)
html_context = {}

try:
    # Google analytics ID, variable must be passed from CI/CD environment    
    html_context['GOOGLE_ANALYTICS_ID'] = os.environ['GOOGLE_ANALYTICS_ID']
    
except KeyError:
    print('No Google analytics environment variable is defined for GOOGLE_ANALYTICS_KEY')
    html_context['GOOGLE_ANALYTICS_ID'] = ''


# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']
