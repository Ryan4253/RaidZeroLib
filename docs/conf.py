DOXYFILE = 'Doxyfile-mcss'


LINKS_NAVBAR1 = [
    ("Get started", 'md_docs_tutorials_setup_setup', []),
    ('Examples', 'md_docs_tutorials_examples_examples', [])
]
LINKS_NAVBAR2 = [
    ('Classes', 'annotated', []),
    ('C++ API', 'md_docs_api_api', [])
]

STYLESHEETS = [
	'./m.css/css/m-dark+documentation.compiled.css',
	'https://fonts.googleapis.com/css?family=Source+Sans+Pro:400,400i,600,600i%7CSource+Code+Pro:400,400i,600'
]

# ALIASES += \
#     "m_div{1}=@xmlonly<mcss:div xmlns:mcss=\"http://mcss.mosra.cz/doxygen/\" mcss:class=\"\1\">@endxmlonly" \
#     "m_enddiv=@xmlonly</mcss:div>@endxmlonly" \
#     "m_span{1}=@xmlonly<mcss:span xmlns:mcss=\"http://mcss.mosra.cz/doxygen/\" mcss:class=\"\1\">@endxmlonly" \
#     "m_endspan=@xmlonly</mcss:span>@endxmlonly" \
#     "m_class{1}=@xmlonly<mcss:class xmlns:mcss=\"http://mcss.mosra.cz/doxygen/\" mcss:class=\"\1\" />@endxmlonly" \
#     "m_footernavigation=@xmlonly<mcss:footernavigation xmlns:mcss=\"http://mcss.mosra.cz/doxygen/\" />@endxmlonly" \
#     "m_examplenavigation{2}=@xmlonly<mcss:examplenavigation xmlns:mcss=\"http://mcss.mosra.cz/doxygen/\" mcss:page=\"\1\" mcss:prefix=\"\2\" />@endxmlonly" \
#     "m_keywords{1}=@xmlonly<mcss:search xmlns:mcss=\"http://mcss.mosra.cz/doxygen/\" mcss:keywords=\"\1\" />@endxmlonly" \
#     "m_keyword{3}=@xmlonly<mcss:search xmlns:mcss=\"http://mcss.mosra.cz/doxygen/\" mcss:keyword=\"\1\" mcss:title=\"\2\" mcss:suffix-length=\"\3\" />@endxmlonly" \
#     "m_enum_values_as_keywords=@xmlonly<mcss:search xmlns:mcss=\"http://mcss.mosra.cz/doxygen/\" mcss:enum-values-as-keywords=\"true\" />@endxmlonly"