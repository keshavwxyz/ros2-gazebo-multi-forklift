from ament_xmllint.main import main
import pytest


@pytest.mark.linter
@pytest.mark.xmllint
def test_xmllint():
    assert main(argv=[]) == 0, 'Found errors'
