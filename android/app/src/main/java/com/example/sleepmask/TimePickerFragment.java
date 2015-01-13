package com.example.sleepmask;

import android.app.Activity;
import android.app.Dialog;
import android.app.TimePickerDialog;
import android.os.Bundle;
import android.support.v4.app.DialogFragment;
import android.text.format.DateFormat;
import android.widget.TimePicker;

import java.util.Calendar;

public class TimePickerFragment extends DialogFragment implements TimePickerDialog.OnTimeSetListener
{
    private TimePickedListener mListener;

    @Override
    public Dialog onCreateDialog(Bundle savedInstanceState)
    {
        // use the current time as the default values for the picker
        final Calendar c = Calendar.getInstance();
        int hour = c.get(Calendar.HOUR_OF_DAY);
        int minute = c.get(Calendar.MINUTE);

        // create a new instance of TimePickerDialog and return it
        return new TimePickerDialog(getActivity(), this, hour, minute, DateFormat.is24HourFormat(getActivity()));
    }

    @Override
    public void onAttach(Activity activity)
    {
        // when the fragment is initially shown (i.e. attached to the activity), cast the activity to the callback interface type
        super.onAttach(activity);
        try
        {
            mListener = (TimePickedListener) activity;
        }
        catch (ClassCastException e)
        {
            throw new ClassCastException(activity.toString() + " must implement " + TimePickedListener.class.getName());
        }
    }

    @Override
    public void onTimeSet(TimePicker view, int hourOfDay, int minute)
    {
        // when the time is selected, send it to the activity via its callback interface method
        Calendar c = Calendar.getInstance();
        c.set(Calendar.HOUR_OF_DAY, hourOfDay);
        c.set(Calendar.MINUTE, minute);

        mListener.onTimePicked(c);
    }

    public static interface TimePickedListener
    {
        public void onTimePicked(Calendar time);
    }
}