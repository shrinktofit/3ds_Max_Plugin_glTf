using System;
using System.IO;
using System.Windows.Forms;

namespace Apricot.Ui {
    public class Win32WindowWrapper: IWin32Window {
        private IntPtr _handle;

        public Win32WindowWrapper(IntPtr hWnd) {
            _handle = (IntPtr)hWnd;
        }

        public IntPtr Handle {
            get { return _handle; }
        }
    }

    public class ExportDialog {
        private Form _form;
        private WebBrowser _browser;
        private bool _shouldExport = false;
        private string _settings = null;

        public Win32WindowWrapper owner = null;

        public ExportDialog(string rootDir) {
            _browser = new WebBrowser();
            _browser.Dock = DockStyle.Fill;
            _browser.IsWebBrowserContextMenuEnabled = false;
            _browser.WebBrowserShortcutsEnabled = false;
            _browser.ObjectForScripting = this;

            _form = new Form();
            _form.Text = "Exporter";
            _form.ShowIcon = false;
            _form.ShowInTaskbar = false;
            _form.Controls.Add(_browser);

            var indexFilePath = Path.Combine(rootDir, "index.html");
            var indexUrl = new Uri(indexFilePath);
            _browser.Navigate(indexUrl);
            _browser.DocumentCompleted += delegate(object sender, WebBrowserDocumentCompletedEventArgs e) {
                System.Console.WriteLine("Document loaded.");
            };
        }

        public void InteropExport() {
            _shouldExport = true;
            _form.Close();
        }

        public void InteropCancel() {
            _form.Close();
        }

        public string Settings {
            get { return _settings;  }
        }
        
        public bool ShouldExport
        {
            get { return _shouldExport;  }
        }

        public void Show() {
            if (owner == null) {
                _form.ShowDialog();
            } else {
                Console.WriteLine(owner.Handle);
                _form.ShowDialog(owner);
            }
        }
    }
}
