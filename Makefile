.PHONY: lint

lint: 
	flake8 --docstring-convention numpy elliptec.py
