import importlib.metadata as im
for d in im.distributions():
    try:
        name = d.metadata['Name']
    except Exception as e:
        print("🔥 Defektes Paket:", getattr(d, '_path', 'unbekannt'))
        print("   Fehler:", repr(e))
