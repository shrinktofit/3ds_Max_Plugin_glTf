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
        private string _appRoot;
        private Form _form;
        private WebBrowser _browser;
        private bool _shouldExport = false;
        private string _settings = null;

        public Win32WindowWrapper owner = null;

        public ExportDialog() {
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

            var appDir = Path.GetDirectoryName(System.Reflection.Assembly.GetExecutingAssembly().CodeBase);
            _appRoot = new Uri(appDir).LocalPath;
            var indexFilePath = Path.Combine(appDir, "Static", "ExportDialog", "index.html");
            var indexUrl = new Uri(indexFilePath);
            _browser.Navigate(indexUrl);
            _browser.DocumentCompleted += delegate(object sender, WebBrowserDocumentCompletedEventArgs e) {
                System.Console.WriteLine("Document loaded.");
            };
        }

        public void InteropDebugPrint(string message) {
            Console.WriteLine(message);
        }

        public void InteropUpdateSettings(string settingsJson) {
            _settings = settingsJson;
        }

        public void InteropExport() {
            _shouldExport = true;
            _form.Close();
        }

        public void InteropCancel() {
            _form.Close();
        }

        public string InteropFetchAppTextFile(string path) {
            var file = Path.Combine(_appRoot, Path.Combine(path.Split('/')));
            Console.WriteLine("Fetch {0}", path);
            var result = System.IO.File.ReadAllText(file);
            return result;
        }

        public string Result {
            get {
                return string.Format(@"
{{
  code: {0},
  settings: {1}
}}", _shouldExport ? 1 : 0, _settings);  }
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
