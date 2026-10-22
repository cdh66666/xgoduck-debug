HLSCL::HLSCL()
{
	End = 0;
}
int HLSCL::EnableTorque(u8 ID, u8 Enable)
{
	return writeByte(ID, HLSCL_TORQUE_ENABLE, Enable);
}
