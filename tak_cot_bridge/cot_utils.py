from datetime import datetime, timedelta

def iso_time_now():
    return datetime.utcnow().isoformat() + "Z"

def iso_time_stale(minutes=5):
    stale = datetime.utcnow() + timedelta(minutes=minutes)
    return stale.isoformat() + "Z"

def build_cot_event(uid, lat, lon, how="m-g", event_type="a-f-G-U-C"):
    """
    Construye un string XML para evento CoT básico de posición.

    Params:
        uid (str): ID único del dispositivo/robot.
        lat (float): Latitud.
        lon (float): Longitud.
        how (str): Método de posicionamiento, default "m-g" (manual GPS).
        event_type (str): Tipo de evento CoT, default "a-f-G-U-C" (unidad terrestre).

    Returns:
        str: XML evento CoT listo para enviar.
    """
    time_now = iso_time_now()
    stale_time = iso_time_stale()

    cot_xml = f"""<event version="2.0" uid="{uid}" type="{event_type}" how="{how}" time="{time_now}" start="{time_now}" stale="{stale_time}" lat="{lat}" lon="{lon}" hae="0" ce="9999999" le="9999999"/>"""
    return cot_xml