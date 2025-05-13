import questionary

questions = [
    {
        'type': 'confirm',
        'name': 'test_success',
        'message': 'War der Test erfolgreich?',
        'default': True
    }
]

answers = {}
answers['test_success'] = questionary.confirm(
    'War der Test erfolgreich?',
    default=True
).ask()

if not answers['test_success']:
    answers['error_message'] = questionary.text(
        'Gib eine Fehlerbeschreibung ein:'
    ).ask()

print(answers)