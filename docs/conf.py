# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Assignment 2'
copyright = '2025, Talha Rebbouh'
author = 'Talha Rebbouh'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
  'sphinx.ext.autodoc', 
'sphinx.ext.doctest', 
'sphinx.ext.intersphinx', 
'sphinx.ext.todo', 
'sphinx.ext.coverage', 
'sphinx.ext.mathjax', 
'sphinx.ext.ifconfig', 
'sphinx.ext.viewcode', 
'sphinx.ext.githubpages', 
"sphinx.ext.napoleon",
 'sphinx.ext.inheritance_diagram',
 'breathe'
]

import os
import sys

sys.path.insert(0, os.path.abspath('home/my_ros/src/assignment_2/scripts'))

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
html_static_path = ['_static']
