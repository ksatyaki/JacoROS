
// This file has been generated by the GUI designer. Do not modify.

public partial class MainWindow
{
	protected virtual void Build ()
	{
		global::Stetic.Gui.Initialize (this);
		// Widget MainWindow
		this.Name = "MainWindow";
		this.Title = global::Mono.Unix.Catalog.GetString ("MainWindow");
		this.WindowPosition = ((global::Gtk.WindowPosition)(4));
		if ((this.Child != null)) {
			this.Child.ShowAll ();
		}
		this.DefaultWidth = 931;
		this.DefaultHeight = 763;
		this.Show ();
		this.DeleteEvent += new global::Gtk.DeleteEventHandler (this.OnDeleteEvent);
	}
}
