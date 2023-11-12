DOXYFILE = 'Doxyfile-mcss'

LINKS_NAVBAR1 = [
    ('Get started', 'md_docs_tutorials_setup_setup', []),
    ('Examples', 'md_docs_tutorials_examples_examples', [])
]

LINKS_NAVBAR2 = [
    ('Classes', 'annotated', []),
    ('C++ API', 'md_docs_api_api', [
        ('Pages', 'pages'),
        ('Setup Guide', 'md_docs_tutorials_setup_setup'),
        ('Namespace', 'namespaceokapi'),
    ])
]

STYLESHEETS = [
	'./m.css/css/m-dark+documentation.compiled.css',
	'https://fonts.googleapis.com/css?family=Source+Sans+Pro:400,400i,600,600i%7CSource+Code+Pro:400,400i,600'
]