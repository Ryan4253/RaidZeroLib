DOXYFILE = 'Doxyfile-mcss'

FAVICON = '../images/raid_zero.png'

LINKS_NAVBAR1 = [
    ('Get Started', 'md_docs_setup_setup', []),
    ('Tutorials', 'md_docs_tutorials_tutorials', [])
]

LINKS_NAVBAR2 = [
    ('Classes', 'annotated', []),
    ('C++ API', 'md_docs_api_api', [
        ('Chassis API', 'md_docs_api_chassis'),
        ('Control API', 'md_docs_api_control'),
        ('Filter API', 'md_docs_api_filter'),
        ('Geometry API', 'md_docs_api_geometry'),
        ('Pathing API', 'md_docs_api_pathing'),
        ('Trajectory API', 'md_docs_api_trajectory'),
        ('Units API', 'md_docs_api_units'),
        ('Utility API', 'md_docs_api_utility'),
        ('Namespace', 'namespacerz')
    ])
]

STYLESHEETS = [
	'./m.css/css/m-dark+documentation.compiled.css',
	'https://fonts.googleapis.com/css?family=Source+Sans+Pro:400,400i,600,600i%7CSource+Code+Pro:400,400i,600',
    'docs.css'
]